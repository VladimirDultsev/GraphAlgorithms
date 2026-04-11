#ifndef ASTAR_H
#define ASTAR_H

#include "../../Core/Solver.h"
#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <map>
#include <vector>

template<typename GraphType>
class AStar : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit AStar(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;
};

template<typename GraphType>
std::vector<typename GraphType::State> AStar<GraphType>::solve(const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    /// Информация о каждой вершине - эвристика, чистое расстояние без эвристики), родитель
    std::unordered_map<State, std::tuple<double, double, State>>pars;

    /// Очередь с приоритетом
    std::map<double, std::unordered_set<State>> dict;

    double finalDist, oldDist, heuristic, newNeighbourDist, currKey;

    /// Рассматриваемая сейчас вершина
    State point = start;

    // Добавляем информацию о стартовой вершине - эвристику, расстояние,
    // за которое мы дошли до неё (0), родитель (она же сама)
    pars[start] = std::make_tuple(graph_.heuristic(start, goal), 0, start);

    // Добавляем стартовую вершину в очередь
    dict[std::get<0>(pars[start])].insert(start);

    // Алгоритм работает пока в очереди есть точки
    while (!dict.empty())
    {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        // Текущая вершина - случайная из наименьших по расстоянию вершин
        point = *((dict.begin())->second).begin();

        // Ключ текущей вершины в очереди
        currKey = dict.begin()->first;

        // Если дощли до финиша
        if (graph_.isGoal(point, goal))
        {
            // Обновляем метрики
            this->updateMetrics();

            // Прекращаем работу
            break;
        }

        // Перебираем соседей вершины
        for (State& neighbour : graph_.getNeighbors(point))
        {
            // Расстояние, за которое мы дошли до этого соседа = расстояние до нас + цена ребра до соседа
            newNeighbourDist = std::get<1>(pars[point]) + graph_.edgeCost(point, neighbour);

            // Если до этого мы не встречали нашего соседа
            if (!pars.count(neighbour))
            {
                // Считаем эвристику до него
                heuristic = graph_.heuristic(neighbour, goal);

                // Обновляем информацию - эвристика, расстояние, за которое дошли до соседа, родитель (мы сами)
                pars[neighbour] = std::make_tuple(heuristic, newNeighbourDist, point);

                // Ключ соседа = расстояние до соседа + эвристика
                finalDist = newNeighbourDist + heuristic;

                // Добавляем вершину в очередь
                dict[finalDist].insert(neighbour);
            }
            // Если найден более короткий путь до соседа
            else if (newNeighbourDist < std::get<1>(pars[neighbour]))
            {
                // Считаем приоритет, под которым сосед был положен в очередь раньше
                oldDist = std::get<0>(pars[neighbour]) + std::get<1>(pars[neighbour]);

                // Удаляем соседа из очереди
                dict[oldDist].erase(neighbour);

                // Если в очереди больше нет точек с таким приоритетом
                if (dict[oldDist].empty()) {
                    // Удаляем этот приоритет из очереди
                    dict.erase(oldDist);
                }

                // Обновляем расстояние до соседа
                std::get<1>(pars[neighbour]) = newNeighbourDist;

                // Теперь мы - родитель этого соседа
                std::get<2>(pars[neighbour]) = point;

                // В очередь кладем соседа под приоритетом в виде суммы расстояния до соседа и эвристики
                finalDist = newNeighbourDist + std::get<0>(pars[neighbour]);
                dict[finalDist].insert(neighbour);
            }
        }

        // Удаляем рассмотренную точку из очереди
        dict[currKey].erase(point);

        // Если в очереди больше нет точек с таким приоритетом
        if (dict[currKey].empty())
        {
            // Удаляем этот приоритет из очереди
            dict.erase(currKey);
        }
    }

    // Восстанавливаем путь
    std::vector<State> path;

    // Прыгаем по родителям пока не дойдём до старта
    for (State i = point; i != start; i = std::get<2>(pars[i]))
    {
        path.push_back(i);
    }
    path.push_back(start);
    // Переворачиваем путь, так как мы шли в обратном порядке (от ребенка к родителю)
    std::reverse(path.begin(), path.end());

    this->updateMetrics();
    return path;
}

#endif
