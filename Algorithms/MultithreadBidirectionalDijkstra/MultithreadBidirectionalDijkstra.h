#ifndef MULTITHREAD_BIDIRECTIONAL_DIJKSTRA_H
#define MULTITHREAD_BIDIRECTIONAL_DIJKSTRA_H

#include "../../Core/Solver.h"
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <algorithm>

template<typename GraphType>
class MultithreadBidirectionalDijkstra : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit MultithreadBidirectionalDijkstra(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;

    /// Очередь с приоритетом для каждого из обходов
    std::map<double, std::unordered_set<State>> dict[2];

    /// Информация о каждой вершине для каждого из обходов - минимальное расстояние,
    /// за которое она была достигнута и родитель
    std::unordered_map<State, std::tuple<double, State>> pars[2];

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

    /// Потокобезопасная модификация алгоритма Дейкстры для запуска в отдельном потоке
    void modifiedDijkstra(int direction, State start, State goal);
};

template<typename GraphType>
void MultithreadBidirectionalDijkstra<GraphType>::modifiedDijkstra(int direction, State start, State goal) {
    // Вычисляем точку старта и финиша данного обхода на основе переданного направления
    State localStart = (direction == 0) ? start : goal;
    State localGoal = (direction == 0) ? goal : start;

    // Добавляем информацию о стартовой вершине - расстояние до неё (0), родитель - она же сама
    pars[direction][localStart] = std::make_tuple( 0.0, localStart);

    // Добавляем старт в очередь
    dict[direction][0.0].insert(localStart);

    // Алгоритм работает пока в очереди есть вершины или пока другой поток не нашёл точку встречи
    while (!dict[direction].empty() && !needToFinish) {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            needToFinish = true;
            cv.notify_all();
            return;
        }

        /// Текущая вершина - случайная из наименьших по расстоянию вершин данного обхода
        State point = *dict[direction].begin()->second.begin();

        /// Приоритет рассматриваемой вершины
        double currentDist = dict[direction].begin()->first;

        {
            // Чтобы использовать потоконебезопасную СД в разных потоках - используем mutex
            std::lock_guard<std::mutex> lock(mtx);

            // Если другой обход уже рассмотрел эту вершину или эта вершина является финишем для нашего конкретного обхода
            if (used.count(point) || (direction == 0 && point == goal) || (direction == 1 && point == start)) {
                // Устанавливаем точку встречи
                meetingPoint = point;

                // Подаём сигнал завершаться второму обходу
                needToFinish = true;

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
                // Добавляем информацию о соседе - эрасстояние за которое мы дошли до него,
                // родитель - мы сами
                pars[direction][neighbour] = std::make_tuple(newDist, point);

                // Кладём вершину в очередь
                dict[direction][newDist].insert(neighbour);
            }
            // Если найден более короткий путь до соседа
            else if (newDist < std::get<1>(pars[direction][neighbour])) {
                /// Приоритет, под которым сосед был положен в очередь раньше
                double oldDist = std::get<1>(pars[direction][neighbour]);

                // Удаляем соседа из словаря
                dict[direction][oldDist].erase(neighbour);

                // Если в словаре больше нет вершин с таким приоритетом
                if (dict[direction][oldDist].empty()) {
                    // Удаляем этот ключ из словаря
                    dict[direction].erase(oldDist);
                }

                // Обновляем расстояние до соседа
                std::get<1>(pars[direction][neighbour]) = newDist;

                // Теперь мы - родитель этого соседа
                std::get<2>(pars[direction][neighbour]) = point;

                // Кладём вершину в очередь под новым приоритетом
                dict[direction][newDist].insert(neighbour);
            }
        }

        // Удаляем вершину из очереди
        dict[direction][currentDist].erase(point);

        // Если в очереди больше нет вершин с таким приоритетом
        if (dict[direction][currentDist].empty()) {
            // Удаляем этот ключ из очереди
            dict[direction].erase(currentDist);
        }
    }
}

template<typename GraphType>
std::vector<typename GraphType::State> MultithreadBidirectionalDijkstra<GraphType>::solve(const State& start, const State& goal) {
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
    std::thread t1(&MultithreadBidirectionalDijkstra::modifiedDijkstra, this, 0, start, goal);
    std::thread t2(&MultithreadBidirectionalDijkstra::modifiedDijkstra, this, 1, start, goal);

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

    // Восстановление пути
    std::vector<State> path;

    // Восстанавливаем часть пути, найденную прямым обходом

    for (State pt = meetingPoint; pt != start; pt = std::get<2>(pars[0][pt])) {
        path.push_back(pt);
    }

    // Добавляем старт в путь
    path.push_back(start);

    // Переворачиваем путь, потому что мы шли в обратном порядке (от ребенка к родителю)
    std::reverse(path.begin(), path.end());

    // Восстанавливаем часть пути, найденную обратным обходом
    if (meetingPoint != goal) {
        // Прыгаем по родителям из pars обратного обхода пока не дойдем до финиша
        for (State pt = std::get<2>(pars[1][meetingPoint]); pt != goal; pt = std::get<2>(pars[1][pt]))
            path.push_back(pt);

        // Добавляем финиш в путь
        path.push_back(goal);
    }

    this->updateMetrics();
    return path;
}

#endif