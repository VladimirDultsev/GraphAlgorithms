#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "../../Core/Solver.h"
#include <vector>
#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <map>

template<typename GraphType>
class Dijkstra : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit Dijkstra(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;
};

template<typename GraphType>
std::vector<typename GraphType::State> Dijkstra<GraphType>::solve(const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    std::unordered_map<State , State> pars1; // родитель
    /// Словарь, в котором для каждой вершины хранится наименьшее расстояние, за которое мы смогли дойти до неё
    std::unordered_map<State, double> m_dist;
    std::map<double, std::unordered_set<State>> dict1;

    // Добаыляем старт в очередь
    dict1[0].insert(start);
    // Расстояние, за которое мы дошли до старта = 0
    m_dist[start] = 0;

    State curr; // Рассматриваемая вершина
    double currDist; // Расстояние, вершины на котором мы сейчас рассматриваем

    while(!dict1.empty()){ // Пока в словаре есть точки
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        // Расстояние до рассматриваемой вершины = наименьшему приоритету очереди
        currDist = dict1.begin()->first;

        // Текущая вершина - случайная из вершин на данном расстоянии
        curr = *dict1.begin()->second.begin();

        // Если вершина, которую мы достали - это финиш, завершаемся
        if(graph_.isGoal(curr, goal)){
            //cout << "solved\n";
            break;
        }

        // Перебираем соседей точки
        for(const State& neighbour : graph_.getNeighbors(curr)){
            // Расстояние, за которое мы дошли до этого соседа = расстояние до нас + цена ребра до соседа
            double newDist = m_dist[curr] + graph_.edgeCost(curr, neighbour);

            // Если мы впервые встречаем данную вершину
            if(m_dist.find(neighbour) == m_dist.end()){
                // Расстояние до неё - это минимальное расстояние, за которое мы дошли до данной вершины + длина ребра до неё
                m_dist[neighbour] = newDist;

                // Добавляем точку в очередь под приоритетом в виде расстояния, за которое мы дошли до неё
                dict1[m_dist[neighbour]].insert(neighbour);

                // Теперь наша вершина - родитель соседа
                pars1[neighbour] = curr;
            }
            // Если мы уже встречали эту вершину, но дошли до неё за меньшее расстояние, чем раньше
            else if(newDist < m_dist[neighbour]){
                // Удаляем эту вершину из очереди под старым приоритетом
                dict1[m_dist[neighbour]].erase(neighbour);

                // Если точек на таком расстоянии больше нет
                if(dict1[m_dist[neighbour]].empty()){
                    // Удаляем этот ключ из словаря
                    dict1.erase(m_dist[neighbour]);
                }

                // Новое расстояние до этой вершины - это минимальное расстояние, за которое мы дошли до данной вершины + длина ребра до неё
                m_dist[neighbour] = newDist;

                // Добавляем точку в словарь под приоритетом в виде расстояния, за которое мы дошли до неё
                dict1[m_dist[neighbour]].insert(neighbour);

                // Теперь наша вершина - родитель найденной нами вершины
                pars1[neighbour] = curr;
            }
        }

        // Удаляем текущую точку из словаря, так ка мы её уже рассмотрели
        dict1[currDist].erase(curr);

        // Если точек на таком расстоянии больше нет
        if(dict1[currDist].empty()){
            // Удаляем этот ключ из словаря
            dict1.erase(currDist);
        }
    }

    // Если финиш не найден
    if(dict1.empty()){
        // Завершаемся с ошибкой
        this->error_ = true;
        this->updateMetrics();
        return {};
    }

    // Восстановление пути
    std::vector<State> path;

    // Прыгаем по родителям, пока не дойдём до старта
    for (State i = curr; i != start; i = pars1[i]){
        path.push_back(i);
    }

    // Добавляем старт в путь
    path.push_back(start);

    // Переворачиваем полученный маршрут, потому что мы шли в обратном порядке (от ребенка к родителю)
    std::reverse(path.begin(), path.end());

    this->updateMetrics();
    return path;
}

#endif
