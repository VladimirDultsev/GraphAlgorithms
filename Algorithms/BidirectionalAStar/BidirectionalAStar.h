#ifndef BIDIRECTIONALASTAR_H
#define BIDIRECTIONALASTAR_H

#include "../../Core/Solver.h"
#include <vector>
#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <map>

template<typename GraphType>
class BidirectionalAStar : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit BidirectionalAStar(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;
};

template<typename GraphType>
std::vector<typename GraphType::State> BidirectionalAStar<GraphType>::solve(const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    /// Индекс алгоритма, который сейчас работает
    int AStarIndex = 0;

    /// Текущая рассматриваемая вершина
    State point = 0;

    double finalDist, oldDist, heuristic, newNeighbourDist;

    /// Очередь с приоритетом для каждого из обходов
    std::map<double, std::unordered_set<State>> dict[2];

    /// Информация о каждой вершине для каждого из обходов - эвристика для неё,
    /// минимальное расстояние, за которое она была достигнута,
    /// родитель, была ли рассмотрена
    std::unordered_map<State, std::tuple<double, double, State, bool>> pars[2];

    // Добавляем информацию о стартовой вершине для прямого обхода
    pars[0][start] = std::make_tuple(graph_.heuristic(start, goal), 0.0, start, false);
    // Добавляем старт в очередь прямого A*-а
    dict[0][std::get<0>(pars[0][start])].insert(start);

    // Добавляем информацию о стартовой вершине для обратного обхода
    pars[1][goal] = std::make_tuple(graph_.heuristic(goal, start), 0.0, goal, false);
    // Добавляем финиш в словарь обратного A*-а
    dict[1][std::get<0>(pars[1][goal])].insert(goal);

    // Алгоритм работает пока хоть в одной из очередей есть вершины
    while (!dict[0].empty() || !dict[1].empty())
    {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        // Если в обоих словарях есть точки
        if (!dict[0].empty() && !dict[1].empty()) {
            // Берем вершину из того словаря, где приоритет меньше
            AStarIndex = (dict[0].begin()->first < dict[1].begin()->first) ? 0 : 1;
        }
        // Если словарь обратного A*-а пустой
        else if (!dict[0].empty()) {
            // Берем вершину из словаря прямого A*-а
            AStarIndex = 0;
        }
        // Если словарь прямого A*-а пустой
        else {
            // Берем вершину из словаря обратного A*-а
            AStarIndex = 1;
        }

        // Текущая вершина - случайная из наименьших по расстоянию вершин выбранного обхода
        point = *(dict[AStarIndex].begin()->second.begin());

        // Если вершина уже удалена из очереди другого обхода, значит A*-ы соединились
        if (pars[1 - AStarIndex].count(point) && std::get<3>(pars[1 - AStarIndex][point])) {
            // Обновляем метрики
            this->updateMetrics();

            // Прекращаем работу
            break;
        }

        // Перебираем соседей вершины
        for (const State& neighbour : graph_.getNeighbors(point))
        {
            // Расстояние, за которое мы дошли до этого соседа = расстояние до нас + цена ребра до соседа
            newNeighbourDist = std::get<1>(pars[AStarIndex][point]) + graph_.edgeCost(point, neighbour);

            // Если до этого мы не встречали нашего соседа
            if (!pars[AStarIndex].count(neighbour))
            {
                // Считаем эвристику до него
                if (AStarIndex == 0) {
                    heuristic = graph_.heuristic(neighbour, goal);
                } else {
                    heuristic = graph_.heuristic(neighbour, start);
                }

                // Добавляем информацию о соседе - эвристика, расстояние за которое мы дошли до него,
                // родитель - мы сами, вершина пока не была посещена
                pars[AStarIndex][neighbour] = std::make_tuple(heuristic, newNeighbourDist, point, false);

                // В очередь кладем вершину под приоритетом в виде суммы расстояния до соседа и эвристики
                finalDist = newNeighbourDist + heuristic;
                dict[AStarIndex][finalDist].insert(neighbour);
            }
            // Если найден более короткий путь до соседа
            else if (newNeighbourDist < std::get<1>(pars[AStarIndex][neighbour]))
            {
                // Считаем приоритет, под которым сосед был положен в очередь раньше
                oldDist = std::get<0>(pars[AStarIndex][neighbour]) +
                          std::get<1>(pars[AStarIndex][neighbour]);

                // Удаляем соседа из словаря
                dict[AStarIndex][oldDist].erase(neighbour);

                // Если в словаре больше нет вершин с таким приоритетом
                if (dict[AStarIndex][oldDist].empty()) {
                    // Удаляем этот ключ из словаря
                    dict[AStarIndex].erase(oldDist);
                }

                // Обновляем расстояние до соседа
                std::get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist;

                // Теперь мы - родитель этого соседа
                std::get<2>(pars[AStarIndex][neighbour]) = point;

                // В очередь кладем вершину под приоритетом в виде суммы расстояния до соседа и эвристики
                finalDist = newNeighbourDist + std::get<0>(pars[AStarIndex][neighbour]);
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }

        // Отмечаем нашу вершину обработанной
        std::get<3>(pars[AStarIndex][point]) = true;

        // Считаем приоритет, под которым наша вершина лежала в очереди
        oldDist = std::get<0>(pars[AStarIndex][point]) +
                  std::get<1>(pars[AStarIndex][point]);

        // Удаляем вершину из очереди
        dict[AStarIndex][oldDist].erase(point);

        // Если в очереди больше нет вершин с таким приоритетом
        if (dict[AStarIndex][oldDist].empty()) {
            // Удаляем этот ключ из очереди
            dict[AStarIndex].erase(oldDist);
        }
    }

    // Восстановление пути
    std::vector<State> path;

    // Прыгаем по родителям из pars прямого A*-а пока не дойдем до старта
    for(State pt = point; pt != start; pt = std::get<2>(pars[0][pt])){
        path.push_back(pt);
    }

    // Добавляем старт в путь
    path.push_back(start);

    // Переворачиваем путь, потому что мы шли в обратном порядке (от ребенка к родителю)
    std::reverse(path.begin(), path.end());

    // Прыгаем по родителям из pars обратного A*-а пока не дойдем до финиша
    for(State pt = std::get<2>(pars[1][point]); pt != goal; pt = std::get<2>(pars[1][pt])){
        path.push_back(pt);
    }

    // Добавляем финиш в путь
    path.push_back(goal);

    this->updateMetrics();
    return path;
}






#include "../../FifteesGraph/fifteesGraph.h"
// Так как для пятнашек используется специфическая эвристика, пришлось написать специализацию алгоритма для этого графа
template<>
std::vector<typename BidirectionalAStar<FifteesGraph>::State> BidirectionalAStar<FifteesGraph>::solve(const State& start, const State& goal) {
    this->resetMetrics();
    this->startTimers();

    int AStarIndex = 0;
    State point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;
    std::map<double, std::unordered_set<State>> dict[2];
    std::unordered_map<State, std::tuple<double, double, State, bool>> pars[2];
    std::vector<State> neighbours;

    std::vector<std::vector<unsigned short>> purpose(2, std::vector<unsigned short>(16));
    std::vector<unsigned short> startField(16);
    FifteesGraph::longToField(start, startField);
    FifteesGraph::calcPurposes(startField, purpose);

    pars[0][start] = std::make_tuple(FifteesGraph::DoubleAStarHeuristic(start, purpose[0]), 0.0, start, false);
    dict[0][std::get<0>(pars[0][start])].insert(start);

    pars[1][goal] = std::make_tuple(FifteesGraph::DoubleAStarHeuristic(goal, purpose[1]), 0.0, goal, false);
    dict[1][std::get<0>(pars[1][goal])].insert(goal);

    while (!dict[0].empty() || !dict[1].empty()) {
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        if (!dict[0].empty() && !dict[1].empty()) {
            AStarIndex = (dict[0].begin()->first < dict[1].begin()->first) ? 0 : 1;
            point = *dict[AStarIndex].begin()->second.begin();
        } else if (!dict[0].empty()) {
            AStarIndex = 0;
            point = *dict[0].begin()->second.begin();
        } else {
            AStarIndex = 1;
            point = *dict[1].begin()->second.begin();
        }

        if (pars[1 - AStarIndex].count(point) && std::get<3>(pars[1 - AStarIndex][point])) {
            break;
        }

        for (const State& neighbour : graph_.getNeighbors(point)) {
            newNeighbourDist = std::get<1>(pars[AStarIndex][point]) + graph_.edgeCost(point, neighbour);

            if (!pars[AStarIndex].count(neighbour)) {
                if (AStarIndex == 0) {
                    heuristic = FifteesGraph::DoubleAStarHeuristic(neighbour, purpose[0]);
                } else {
                    heuristic = FifteesGraph::DoubleAStarHeuristic(neighbour, purpose[1]);
                }
                pars[AStarIndex][neighbour] = std::make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic;
                dict[AStarIndex][finalDist].insert(neighbour);
            } else if (newNeighbourDist < std::get<1>(pars[AStarIndex][neighbour])) {
                oldDist = std::get<0>(pars[AStarIndex][neighbour]) + std::get<1>(pars[AStarIndex][neighbour]);
                dict[AStarIndex][oldDist].erase(neighbour);
                if (dict[AStarIndex][oldDist].empty()) {
                    dict[AStarIndex].erase(oldDist);
                }
                std::get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist;
                std::get<2>(pars[AStarIndex][neighbour]) = point;
                finalDist = newNeighbourDist + std::get<0>(pars[AStarIndex][neighbour]);
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }

        std::get<3>(pars[AStarIndex][point]) = true;
        oldDist = std::get<0>(pars[AStarIndex][point]) + std::get<1>(pars[AStarIndex][point]);
        dict[AStarIndex][oldDist].erase(point);
        if (dict[AStarIndex][oldDist].empty()) {
            dict[AStarIndex].erase(oldDist);
        }
    }

    // Восстановление пути
    std::vector<State> path;
    for (State pt = point; pt != start; pt = std::get<2>(pars[0][pt])) {
        path.push_back(pt);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    for (State pt = std::get<2>(pars[1][point]); pt != goal; pt = std::get<2>(pars[1][pt])) {
        path.push_back(pt);
    }
    path.push_back(goal);

    this->updateMetrics();
    return path;
}

#endif
