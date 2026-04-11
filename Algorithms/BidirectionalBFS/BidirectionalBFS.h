#ifndef BIDIRECTIONAL_BFS_H
#define BIDIRECTIONAL_BFS_H

#include "../../Core/Solver.h"
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <ctime>

template<typename GraphType>
class BidirectionalBFS : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;

    explicit BidirectionalBFS(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;
};

template<typename GraphType>
std::vector<typename GraphType::State> BidirectionalBFS<GraphType>::solve(const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    /// Очередь вершин на рассмотрение для каждого обхода
    std::queue<State> q[2];

    /// Для каждого обхода информация о каждой вершине - кратчайшее расстояние от старта до неё,
    /// была ли посещена, родитель
    std::unordered_map<State, std::tuple<unsigned long long, bool, State>> pars[2];

    // Добавляем старт в очередь прямого обхода
    q[0].push(start);

    // Информация о стартовой вершине: расстояние 0, уже была посещёна, родитель она же сама
    pars[0][start] = std::make_tuple(0, true, start);

    // Добавляем финиш в очередь обратного обхода
    q[1].push(goal);

    // Информация о финишной вершине: расстояние 0, уже была посещёна, родитель она же сама
    pars[1][goal] = std::make_tuple(0, true, goal);

    /// Индекс алгоритма, который сейчас работает
    int bfsIndex = 0;

    /// Точка, в которой встретились алгоритмы
    State meetingPoint = start;

    // Алгоритм работает пока хотя бы в одной очереди есть вершины
    while (!q[0].empty() || !q[1].empty()) {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        // Меняем обход на противоположный предыдущему
        bfsIndex = 1 - bfsIndex;

        // Достаём первую вершину из очереди выбранного обхода
        State current = q[bfsIndex].front();
        q[bfsIndex].pop();

        // Проверяем, посещена ли эта точка с другой стороны
        if (pars[1 - bfsIndex].count(current) && std::get<1>(pars[1 - bfsIndex][current])) {
            // Если да - отмечаем её точкой встречи и завершаемся
            meetingPoint = current;
            break;
        }

        // Если вдруг один алгоритм сам дошёл до своей цели
        if ((bfsIndex == 0 && graph_.isGoal(current, goal)) ||
            (bfsIndex == 1 && graph_.isGoal(current, start))) {
            // Отмечаем ткущую вершину точкой встречи и завершаемся
            meetingPoint = current;
            break;
        }

        // Перебираем соседей вершины
        for (const State& neighbour : graph_.getNeighbors(current)) {
            // Если до этого сосед не был посещён
            if (!std::get<1>(pars[bfsIndex][neighbour])) {
                // Расстояние до него = расстояние до current + 1 (так как граф невзвешенный)
                unsigned long long newDist = std::get<0>(pars[bfsIndex][current]) + 1;

                // Записываем вычисленное расстояние, вершина уже была посещена, родитель - current
                pars[bfsIndex][neighbour] = std::make_tuple(newDist, true, current);

                // Добавляем вершину в очередь
                q[bfsIndex].push(neighbour);
            }
        }
    }

    // Если обходы не встретились
    if(q[0].empty() && q[1].empty()) {
        // Завершаемся с ошибкой
        this->error_ = true;
        this->updateMetrics();
        return {};
    }

    // Восстановление пути
    std::vector<State> path;
    // Прыгаем по соседям из pars прямого обхода пока не дойдем до старта
    for(State p = meetingPoint; p != start; p = std::get<2>(pars[0][p])){
        path.push_back(p);
    }

    // Добавляем старт в путь
    path.push_back(start);

    // Переворачиваем путь, потому что мы шли в обратном порядке (от ребенка к родителю)
    std::reverse(path.begin(), path.end());

    // Прыгаем по родителям из pars обратного обхода пока не дойдем до финиша
    for(State p = std::get<2>(pars[1][meetingPoint]); p != goal; p = std::get<2>(pars[1][p])){
        path.push_back(p);
    }

    // Добавляем финиш в путь
    path.push_back(goal);

    this->updateMetrics();
    return path;
}

#endif