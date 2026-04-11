#ifndef BIDIRECIONALDIJKSTRA_H
#define BIDIRECIONALDIJKSTRA_H

#include "../../Core/Solver.h"
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <map>
#include <unordered_map>

template<typename GraphType>
class BidirectionalDijkstra : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit BidirectionalDijkstra(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;
};

template<typename GraphType>
std::vector<typename GraphType::State> BidirectionalDijkstra<GraphType>::solve(const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    /// Индекс алгоритма, который сейчас работает
    int direction = 0;

    /// Точка, в которой встретились алгоритмы
    State meetingPoint = start;

    double newDist, oldDist, currDist;

    /// Очередь с приоритетом
    std::map<double, std::unordered_set<State>> dict[2];

    /// Информация о каждой вершине для каждого из обходов - минимальное расстояние,
    /// за которое она была достигнута, родитель, была ли рассмотрена
    std::unordered_map<State, std::tuple<double, State, bool>> pars[2];

    // Добавляем информацию о стартовой вершине для прямого обхода
    pars[0][start] = std::make_tuple(0.0, start, false);
    // Добавляем старт в очередь прямого обхода
    dict[0][0.0].insert(start);

    // Добавляем информацию о стартовой вершине для обратного обхода
    pars[1][goal] = std::make_tuple(0.0, goal, false);
    // Добавляем финиш в словарь обратного обхода
    dict[1][0.0].insert(goal);

    // Алгоритм работает пока хоть в одной из очередей есть вершины
    while (!dict[0].empty() || !dict[1].empty()) {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        // Если в обоих словарях есть точки
        if (!dict[0].empty() && !dict[1].empty()) {
            // Берем вершину из того словаря, где приоритет меньше
            direction = (dict[0].begin()->first < dict[1].begin()->first) ? 0 : 1;
        }
        // Если словарь обратного обхода пустой
        else if (!dict[0].empty()) {
            // Берем вершину из словаря прямого обхода
            direction = 0;
        }
        // Если словарь прямого обхода пустой
        else {
            // Берем вершину из словаря обратного обхода
            direction = 1;
        }

        // Ключ, под которым лежит текущая вершина
        currDist = dict[direction].begin()->first;

        /// Текущая вершина - случайная из наименьших по расстоянию точек выбранного обхода
        State current = *dict[direction].begin()->second.begin();

        // Если точка уже удалена из очереди другого обхода, значит обходы соединились
        if (pars[1 - direction].count(current) && std::get<2>(pars[1 - direction][current])) {
            meetingPoint = current;
            break;
        }

        // Перебираем соседей вершины
        for (const State& neighbor : graph_.getNeighbors(current)) {
            // Расстояние, за которое мы дошли до этого соседа = расстояние до нас + цена ребра до соседа
            newDist = std::get<0>(pars[direction][current]) + graph_.edgeCost(current, neighbor);

            // Если до этого мы не встречали нашего соседа
            if (!pars[direction].count(neighbor)) {
                // Добавляем информацию о вершине - расстояние за которое мы дошли до неё,
                // родитель - мы сами, вершина пока не была посещена
                pars[direction][neighbor] = std::make_tuple(newDist, current, false);

                // Кладём вершину в очередь
                dict[direction][newDist].insert(neighbor);
            }
            // Если найден более короткий путь до соседа
            else if (newDist < std::get<0>(pars[direction][neighbor])) {
                // Считаем приоритет, под которым сосед был положен в словарь раньше
                oldDist = std::get<0>(pars[direction][neighbor]);

                // Удаляем соседа из словаря
                dict[direction][oldDist].erase(neighbor);

                // Если в словаре больше нет вершин с таким приоритетом
                if (dict[direction][oldDist].empty()) {
                    // Удаляем этот ключ из словаря
                    dict[direction].erase(oldDist);
                }

                // Обновляем расстояние до соседа
                std::get<0>(pars[direction][neighbor]) = newDist;

                // Теперь мы - родитель этого соседа
                std::get<1>(pars[direction][neighbor]) = current;

                // Кладём вершину в очередь под новым приоритетом
                dict[direction][newDist].insert(neighbor);
            }
        }
        // Отмечаем нашу вершину обработанной
        std::get<2>(pars[direction][current]) = true;

        // Удаляем вершину из очереди
        dict[direction][currDist].erase(current);

        // Если в очереди больше нет вершин с таким приоритетом
        if (dict[direction][currDist].empty()) {
            // Удаляем этот ключ из очереди
            dict[direction].erase(currDist);
        }
    }

    // Если обходы не встретились
    if(dict[0].empty() &&dict[1].empty()) {
        // Завершаемся с ошибкой
        this->error_ = true;
        this->updateMetrics();
        return {};
    }

    // Восстановление пути
    std::vector<State> path;

    // Прыгаем по родителям из pars прямого обхода пока не дойдем до старта
    for(State current = meetingPoint; current != start; current = std::get<1>(pars[0][current])){
        path.push_back(current);
    }

    // Добавляем старт в путь
    path.push_back(start);

    // Переворачиваем путь, потому что мы шли в обратном порядке (от ребенка к родителю)
    std::reverse(path.begin(), path.end());

    // Прыгаем по родителям из pars обратного обхода пока не дойдем до финиша
    for(State current = std::get<1>(pars[1][meetingPoint]); current != goal; current = std::get<1>(pars[1][current])){
        path.push_back(current);
    }

    // Добавляем финиш в путь
    path.push_back(goal);

    this->updateMetrics();
    return path;
}

#endif
