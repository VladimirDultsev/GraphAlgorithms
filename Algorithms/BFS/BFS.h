#ifndef BFS_H
#define BFS_H

#include "../../Core/Solver.h"
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>

template<typename GraphType>
class BFS : public core::Solver<GraphType> {
public:
    using State = typename GraphType::State;
    explicit BFS(const GraphType& graph) : graph_(graph) {}

    std::vector<State> solve(const State& start, const State& goal) override;

private:
    const GraphType& graph_;
};

template<typename GraphType>
std::vector<typename GraphType::State> BFS<GraphType>::solve(const State& start, const State& goal) {
    // Сбрасываем счетчики времени выполнения
    this->resetMetrics();
    // Начинаем замер
    this->startTimers();

    /// Информация о каждой вершине - кратчайшее расстояние от старта до неё,
    /// была ли посещена, родитель
    std::unordered_map<State, std::tuple<unsigned long long, bool, State>> pars;

    /// Текущая рассматриваемая вершина
    State curr;

    /// Очередь вершин на рассмотрение
    std::queue<State> q;

    // Добавляем старт в очередь
    q.push(start);

    // Информация о стартовой вершине: расстояние 0, уже была посещёна, родитель она же сама
    pars[start] = std::make_tuple(0, true, start);

    // Алгоритм работает пока в очереди есть вершины
    while(!q.empty())
    {
        // Проверяем допустимость времени выполнения
        if (this->checkTimeout()) {
            this->updateMetrics();
            return {};
        }

        // Достаём первую вершину из очереди
        curr = q.front();

        // Если эта вершина - финиш, завершаемся
        if(curr == goal)
        {
            //cout<<"solved"<<'\n';
            break;
        }

        q.pop();

        // Перебираем соседей вершины
        for(const State& neighbour : graph_.getNeighbors(curr))
        {
            // Если сосед до этого не был посещён
            if(!std::get<1>(pars[neighbour]))
            {
                // Расстояние до него = расстояние до curr + 1 (так как граф невзвешенный),
                // теперь curr - родитель neighbour
                pars[neighbour] = std::make_tuple(std::get<0>(pars[curr]) + 1, true, curr);

                // Добавляем в очередь
                q.push(neighbour);
            }
        }
    }

    // Если финиш не был достигнут
    if(!pars.count(goal)) {
        // Завершаемся с ошибкой
        this->error_ = true;
        this->updateMetrics();
        return {};
    }

    // Начинаем восстановление пути
    std::vector<State> lst;

    // Прыгаем по родителям начиная с финиша, пока не дойдём до старта
    for(State point = goal; point != start; point = std::get<2>(pars[point])) {
        lst.push_back(point);
    }
    // Добавляем старт в путь
    lst.push_back(start);

    // Переворачиваем путь так как шли в обратном порядке (от ребенка к родителю)
    std::reverse(lst.begin(), lst.end());

    this->updateMetrics();
    return lst;
}

#endif