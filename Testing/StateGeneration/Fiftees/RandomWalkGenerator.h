#ifndef UNIFIEDGRAPHALGORITHMS_RANDOMWALKGENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_RANDOMWALKGENERATOR_H

#include <random>
#include <vector>
#include <memory>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <iostream>
#include <stack>
#include "../ITestGenerator.h"
#include "BaseFifteesGenerator.h"
#include "../../../Algorithms/BidirectionalAStar/BidirectionalAStar.h"

template<typename GraphType>
class RandomWalk : public IFifteesGeneratorType<GraphType> {
public:
    using State = typename GraphType::State;

    explicit RandomWalk()= default;

    State generate(const GraphType& graph, State goal, int steps) override {
        std::unordered_map<State, bool> visited;
        State buff = goal, lastVisited;
        std::stack<State> path;
        unsigned char rnd;
        bool flag = true;
        std::vector<State> neighbours;

        for(int i = 0; i < steps;)
        {
            if(!flag)
            {
                visited[buff] = false;
                buff = path.top();
                path.pop();
                --i;
            }
            else
            {
                ++i;
            }

            neighbours = graph.getNeighbors(buff);
            flag = false;
            for(int count = 0; count < neighbours.size(); ++count)
            {
                rnd = rand() % neighbours.size();
                if (!visited.count(neighbours[rnd]) || !visited[neighbours[rnd]])
                {
                    visited[neighbours[rnd]] = true;
                    path.push(buff);
                    buff = neighbours[rnd];
                    flag = true;
                    break;
                }
            }
        }
        return buff;
    }
};

#endif