#ifndef MULTITHREAD_BIDIRECTIONAL_BFS_H
#define MULTITHREAD_BIDIRECTIONAL_BFS_H

#include "../../Core/Solver.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <vector>
#include <algorithm>
#include <ctime>

template<typename GraphType>
class MultithreadBidirectionalBFS : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;

    explicit MultithreadBidirectionalBFS(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;

    /// Очередь вершин на рассмотрение для каждого из обходов
    std::queue<State> q[2];

    /// Для каждого обхода информация о каждой вершине - кратчайшее расстояние от старта до неё,
    /// была ли посещена, родитель
    std::unordered_map<State, std::tuple<unsigned long long, bool, State>> pars[2];

    /// Общая для обоих обходов структура данных - хранит обработанные вершины
    std::unordered_set<State> used;

    /// Мьютекс для безопасного обновления рассмотренных вершин
    std::mutex mtx;

    /// Мьютекс блокировки основного потока на время работы обходов
    std::mutex cv_mtx;

    /// Condition variable для блокировки основного потока на время работы обходов
    std::condition_variable cv;

    /// Если какой-то из обходов обнаружил что обходы встретились - устанавливает единицу
    std::atomic<bool> needToFinish{false};

    /// Точка встречи обходов
    State meetingPoint;

    /// Потокобезопасная модификация BFS для запуска в отдельном потоке
    void modifiedBFS(int direction, State start, State goal);
};

template<typename GraphType>
void MultithreadBidirectionalBFS<GraphType>::modifiedBFS(int direction, State start, State goal) {
    // Вычисляем точку старта и финиша данного обхода на основе переданного направления
    State localStart = (direction == 0) ? start : goal;
    State localGoal  = (direction == 0) ? goal  : start;

    // Добавляем старт в очередь
    q[direction].push(localStart);

    // Информация о стартовой вершине: расстояние 0, уже была посещёна, родитель она же сама
    pars[direction][localStart] = std::make_tuple(0U, true, localStart);

    // Алгоритм работает пока в очереди есть вершины или пока другой поток не нашёл точку встречи
    while (!q[direction].empty() && !needToFinish) {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            needToFinish = true;
            cv.notify_all();
            return;
        }

        // Достаём первую вершину из очереди
        State current = q[direction].front();
        q[direction].pop();

        {
            // Чтобы использовать потоконебезопасную СД в разных потоках - используем mutex
            std::lock_guard<std::mutex> lock(mtx);

            // Если другой обход уже рассмотрел эту вершину или эта вершина является финишем для нашего конкретного обхода
            if (used.count(current) || (direction == 0 && graph_.isGoal(current, goal)) ||
                                       (direction == 1 && graph_.isGoal(current, start))) {
                // Устанавливаем точку встречи
                meetingPoint = current;

                // Подаём сигнал завершаться второму обходу
                needToFinish = true;

                // Пробуждаем основной поток
                cv.notify_all();
                return;
            }

            // Добавляем нашу вершину в рассмотренные
            used.insert(current);
        }

        // Перебираем соседей вершины
        for (const State& neighbour : graph_.getNeighbors(current)) {
            // Если сосед до этого не был посещён
            if(!std::get<1>(pars[neighbour]))
            {
                // Расстояние до него = расстояние до current + 1 (так как граф невзвешенный),
                // теперь current - родитель neighbour
                pars[neighbour] = std::make_tuple(std::get<0>(pars[current]) + 1, true, current);

                // Добавляем в очередь
                q.push(neighbour);
            }
        }
    }
}

template<typename GraphType>
std::vector<typename GraphType::State> MultithreadBidirectionalBFS<GraphType>::solve(
        const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    // Очищаем все структуры данных
    q[0] = {}; q[1] = {};
    pars[0].clear(); pars[1].clear();
    used.clear();
    needToFinish = false;
    meetingPoint = start;

    // Запускаем прямой и обратный обходы в своих потоках
    std::thread t1(&MultithreadBidirectionalBFS::modifiedBFS, this, 0, start, goal);
    std::thread t2(&MultithreadBidirectionalBFS::modifiedBFS, this, 1, start, goal);

    {
        // Ждём завершения одного из алгоритмов
        std::unique_lock<std::mutex> lk(cv_mtx);
        cv.wait(lk, [this] { return needToFinish.load(); });
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
    // Прыгаем по родителям из pars прямого обхода пока не дойдем до старта
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