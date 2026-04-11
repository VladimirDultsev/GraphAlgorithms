#ifndef UNIFIEDGRAPHALGORITHMS_HYBRID_FIFTEES_GENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_HYBRID_FIFTEES_GENERATOR_H

#include <random>
#include <vector>
#include <memory>
#include "../ITestGenerator.h"
#include "BaseFifteesGenerator.h"

template<typename GraphType>
class HybridStrategy : public IFifteesGeneratorType<GraphType> {
public:
    using State = typename GraphType::State;

    explicit HybridStrategy(double greedyProb = 0.7)
            : greedyProb_(greedyProb), rng_(std::random_device{}()) {}

    State generate(const GraphType& graph, State goal, int steps) override {
        if (steps <= 0) return goal;

        State current = goal;
        State prev = goal;
        std::uniform_real_distribution<double> probDist(0.0, 1.0);

        for (int step = 0; step < steps; ++step) {
            auto neighbors = graph.getNeighbors(current);
            if (neighbors.empty()) break;

            std::vector<State> avMoves;
            for (State nb : neighbors) {
                if (nb != prev) {
                    avMoves.push_back(nb);
                }
            }

            if (avMoves.empty()) {
                avMoves = neighbors;
            }

            State next;
            // Выбираем какой шаг сделать следующим
            double r = probDist(rng_);

            if (r < greedyProb_ && avMoves.size() > 1) {
                // Жадный выбор (максимизируем эвристику)
                double bestHeuristic = -1.0;
                State bestState = avMoves[0];
                for (State nb : avMoves) {
                    double heuristic = graph.heuristic(nb, goal);
                    if (heuristic > bestHeuristic) {
                        bestHeuristic = heuristic;
                        bestState = nb;
                    }
                }
                next = bestState;
            } else {
                // Случайный выбор
                std::uniform_int_distribution<size_t> idx(0, avMoves.size() - 1);
                next = avMoves[idx(rng_)];
            }

            prev = current;
            current = next;
        }
        return current;
    }

private:
    double greedyProb_;
    std::mt19937 rng_;
};

#endif