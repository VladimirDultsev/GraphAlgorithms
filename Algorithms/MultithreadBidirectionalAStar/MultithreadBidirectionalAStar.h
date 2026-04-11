#ifndef MULTITHREAD_BIDIRECTIONAL_ASTAR_H
#define MULTITHREAD_BIDIRECTIONAL_ASTAR_H

#include "../../Core/Solver.h"
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <algorithm>
#include <algorithm>
#include <cmath>
#include <iostream>

template<typename GraphType>
class MultithreadBidirectionalAStar : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit MultithreadBidirectionalAStar(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;

    /// Очередь с приоритетом для каждого из обходов
    std::map<double, std::unordered_set<State>> dict[2];

    /// Информация о каждой вершине для каждого из обходов - эвристика для неё,
    /// минимальное расстояние, за которое она была достигнута,
    /// родитель, была ли рассмотрена
    std::unordered_map<State, std::tuple<double, double, State>> pars[2];

    /// Мьютекс для безопасного обновления рассмотренных вершин
    std::mutex mtx;

    /// Общая для обоих обходов структура данных - хранит обработанные вершины
    std::unordered_set<State> used;

    /// Если какой-то из обходов обнаружил что обходы встретились - устанавливает единицу
    std::atomic<bool> needToFinish{false};

    /// Мьютекс блокировки основного потока на время работы обходов
    std::mutex cv_mtx;

    /// Condition variable для блокировки основного потока на время работы обходов
    std::condition_variable cv;

    /// Точка встречи обходов
    State meetingPoint;

    /// Потокобезопасная модификация A* для запуска в отдельном потоке
    void modifiedAStar(int direction, State start, State goal);
};

template<typename GraphType>
void MultithreadBidirectionalAStar<GraphType>::modifiedAStar(int direction, State start, State goal) {
    // Вычисляем точку старта и финиша данного обхода на основе переданного направления
    State localStart = (direction == 0) ? start : goal;
    State localGoal  = (direction == 0) ? goal  : start;

    // Считаем эвристику для стартовой вершины
    double heuristic = graph_.heuristic(localStart, localGoal);

    // Добавляем информацию о стартовой вершине - эвристику, расстояние (0), родитель - она же сама
    pars[direction][localStart] = std::make_tuple(heuristic, 0.0, localStart);
    // Добавляем точку старта в очередь
    dict[direction][heuristic].insert(localStart);

    // Алгоритм работает пока в очереди есть вершины или пока другой поток не нашёл точку встречи
    while (!dict[direction].empty() && !needToFinish && !this->error_) {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            needToFinish = true;
            this->error_ = true;
            cv.notify_all();
            return;
        }

        /// Текущая вершина - случайная из наименьших по расстоянию вершин данного обхода
        State point = *dict[direction].begin()->second.begin();

        /// Приоритет рассматриваемой вершины
        double currentPriority = dict[direction].begin()->first;

        {
            // Чтобы использовать потоконебезопасную СД в разных потоках - используем mutex
            std::lock_guard<std::mutex> lock(mtx);

            // Если другой обход уже рассмотрел эту вершину или эта вершина является финишем для нашего конкретного обхода
            if (used.count(point) || (direction == 0 && point == goal) || (direction == 1 && point == start)) {
                // Устанавливаем точку встречи
                meetingPoint = point;

                // Подаём сигнал завершаться второму обходу
                needToFinish.store(true, std::memory_order_release);

                // Пробуждаем основной поток
                cv.notify_all();
                return;
            }

            // Добавляем нашу вершину в рассмотренные
            used.insert(point);
        }

        // Перебираем соседей вершины
        for (const State& neighbour : graph_.getNeighbors(point)) {
            /// Расстояние, за которое мы дошли до этого соседа = расстояние до нас + цена ребра до соседа
            double newDist = std::get<1>(pars[direction][point]) + graph_.edgeCost(point, neighbour);

            // Если до этого мы не встречали нашего соседа
            if (!pars[direction].count(neighbour)) {
                // Считаем эвристику до него
                heuristic = graph_.heuristic(neighbour, localGoal);

                // Добавляем информацию о соседе - эвристика, расстояние за которое мы дошли до него,
                // родитель - мы сами, вершина пока не была посещена
                pars[direction][neighbour] = std::make_tuple(heuristic, newDist, point);

                // В очередь кладем вершину под приоритетом в виде суммы расстояния до соседа и эвристики
                double priority = newDist + heuristic;
                dict[direction][priority].insert(neighbour);
            }
            // Если найден более короткий путь до соседа
            else if (newDist < std::get<1>(pars[direction][neighbour])) {
                /// Считаем приоритет, под которым сосед был положен в очередь раньше
                double oldPriority = std::get<0>(pars[direction][neighbour]) + std::get<1>(pars[direction][neighbour]);

                // Удаляем соседа из словаря
                dict[direction][oldPriority].erase(neighbour);

                // Если в словаре больше нет вершин с таким приоритетом
                if (dict[direction][oldPriority].empty()){
                    // Удаляем этот ключ из словаря
                    dict[direction].erase(oldPriority);
                }

                // Обновляем расстояние до соседа
                std::get<1>(pars[direction][neighbour]) = newDist;

                // Теперь мы - родитель этого соседа
                std::get<2>(pars[direction][neighbour]) = point;

                // В очередь кладем вершину под приоритетом в виде суммы расстояния до соседа и эвристики
                double newPriority = newDist + std::get<0>(pars[direction][neighbour]);
                dict[direction][newPriority].insert(neighbour);
            }
        }

        // Удаляем вершину из очереди
        dict[direction][currentPriority].erase(point);

        // Если в очереди больше нет вершин с таким приоритетом
        if (dict[direction][currentPriority].empty()){
            // Удаляем этот ключ из очереди
            dict[direction].erase(currentPriority);
        }
    }
}

template<typename GraphType>
std::vector<typename GraphType::State> MultithreadBidirectionalAStar<GraphType>::solve(
        const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    // Очищаем все структуры данных
    dict[0].clear();
    dict[1].clear();
    pars[0].clear();
    pars[1].clear();
    used.clear();
    needToFinish = false;
    meetingPoint = start;

    // Запускаем прямой и обратный обходы в своих потоках
    std::thread t1(&MultithreadBidirectionalAStar::modifiedAStar, this, 0, start, goal);
    std::thread t2(&MultithreadBidirectionalAStar::modifiedAStar, this, 1, start, goal);

    {
        // Ждём завершения одного из алгоритмов
        std::unique_lock<std::mutex> lk(cv_mtx);
        cv.wait(lk, [this]{ return needToFinish.load(); });
    }

    // Ждём завершения всех потоков и освобождения ресурсов
    if (t1.joinable()) t1.join();
    if (t2.joinable()) t2.join();

    // Если упёрлись в таймаут
    if (this->error_) {
        this->updateMetrics();

        // Завершаемся
        return {};
    }

    // Восстанавливаем путь
    std::vector<State> path;

    // Восстанавливаем часть пути, найденную прямым обходом
    if (pars[0].count(meetingPoint)) {
        // Прыгаем по родителям из pars прямого A*-а пока не дойдем до старта
        for (State pt = meetingPoint; pt != start; pt = std::get<2>(pars[0][pt]))
            path.push_back(pt);

        // Добавляем старт в путь
        path.push_back(start);

        // Переворачиваем путь, потому что мы шли в обратном порядке (от ребенка к родителю)
        std::reverse(path.begin(), path.end());
    }

    // Восстанавливаем часть пути, найденную обратным обходом
    if (meetingPoint != goal && pars[1].count(meetingPoint)) {
        // Прыгаем по родителям из pars прямого A*-а пока не дойдем до финиша
        for (State pt = std::get<2>(pars[1][meetingPoint]); pt != goal; pt = std::get<2>(pars[1][pt]))
            path.push_back(pt);

        // Добавляем финиш в путь
        path.push_back(goal);
    }

    this->updateMetrics();
    return path;
}

#endif