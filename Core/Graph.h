#ifndef CORE_GRAPH_H
#define CORE_GRAPH_H

#include <vector>

namespace core {
    template <typename State>
    class Graph {
    public:
        virtual ~Graph() = default;

        /// Возвращает список соседей данной вершины
        virtual std::vector<State> getNeighbors(const State& state) const = 0;

        /// Считает эвристику между двумя точками
        virtual double heuristic(const State& from, const State& to) const = 0;

        /// Проверяет является ли точка финишем
        virtual bool isGoal(const State& current, const State& goal) const = 0;

        /// Возвращает стоимость перехода между from и to
        virtual double edgeCost(const State& from, const State& to) const {
            return 1.0;
        }
    };

}

#endif