#ifndef UNIFIEDGRAPHALGORITHMS_BFSBASEDGENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_BFSBASEDGENERATOR_H

#include <random>
#include <vector>
#include <memory>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "../ITestGenerator.h"
#include "BaseFifteesGenerator.h"
#include "../../../Algorithms/BidirectionalAStar/BidirectionalAStar.h"

template<typename GraphType>
class BFSBased : public IFifteesGeneratorType<GraphType> {
public:
    using State = typename GraphType::State;

    explicit BFSBased() = default;

    State generate(const GraphType& graph, State goal, int steps) override {
        std::random_device rd;
        std::mt19937 rng(rd());
        std::unordered_map<State, int> dst;
        std::queue<State> q;
        std::vector<State> targetStates;
        std::vector<State> neighbours;
        BidirectionalAStar<FifteesGraph> doubleAstar(graph);

        dst[goal] = 0;
        q.push(goal);
        while (!q.empty()) {
            State current = q.front();
            q.pop();

            int currentDist = dst[current];

            if (currentDist == steps) {
                targetStates.push_back(current);
                if (targetStates.size() > 1000) break;
                continue;
            }

            if (currentDist > steps) continue;

            for (auto& neighbor : graph.getNeighbors(current)) {
                if (dst.find(neighbor) == dst.end()) {
                    dst[neighbor] = currentDist + 1;
                    q.push(neighbor);
                }
            }
        }

        if (!targetStates.empty()) {
            std::uniform_int_distribution<unsigned long> dist(0, targetStates.size() - 1);
            return targetStates[dist(rng)];
        }
        std::cout << "States on distance " << steps << " not found" << '\n';
        return goal;
    }
};

#endif
