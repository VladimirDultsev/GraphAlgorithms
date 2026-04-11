#ifndef FIFTEEN_GRAPH_H
#define FIFTEEN_GRAPH_H

#include "../Core/Graph.h"
#include <cstdint>
#include <vector>

class FifteesGraph : public core::Graph<uint64_t> {
    public:
        using State = uint64_t;
        explicit FifteesGraph(uint64_t goal);

        /// Возвращает список соседей данной вершины
        [[nodiscard]] std::vector<uint64_t> getNeighbors(const uint64_t& state) const override;

        /// Считает эвристику между двумя точками
        [[nodiscard]] double heuristic(const uint64_t& state, const uint64_t& goal) const override;

        /// Проверяет является ли точка финишем
        [[nodiscard]] bool isGoal(const uint64_t& state, const uint64_t& goal) const override;

        /// Возвращает стоимость перехода между from и to
        [[nodiscard]] double edgeCost(const uint64_t& from, const uint64_t& to) const override {
            return 1.0; // Так как граф невзвешенный
        }

        /// Раскодирует состояние поля num в массив f
        static void longToField(uint64_t num, std::vector<unsigned short>& f);

        /// Возвращает закодированное в число состояние поля f
        static uint64_t fieldToLong(const std::vector<unsigned short>& f);

        /// Возвращает поле st с поменянными местами ячейками на позициях i и zeroPos
        static uint64_t swap(uint64_t st, unsigned short i, unsigned short zeroPos);

        /// Эвристика для 2A*
        static double DoubleAStarHeuristic(uint64_t f, const std::vector<unsigned short>& Purpose);

        /// Вспомогательная функция маппинга для эвристики 2A*
        static void calcPurposes(const std::vector<unsigned short> &st, std::vector<std::vector<unsigned short>>& Purpose);

    private:
        uint64_t goal_;
    };
#endif