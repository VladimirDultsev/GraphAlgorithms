#include "fiftees_generator.h"
#include "../algorithms/bidirectional_astar.h"
#include <iostream>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include "graph.h"
using namespace Graph;

unsigned long long int fifteesGenerator(int targetDistance) {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::unordered_map<unsigned long long int, int> distances;
    std::queue<unsigned long long int> q;
    std::vector<unsigned long long int> statesAtTargetDistance;
    std::vector<unsigned long long int> neighbours;

    distances[finish] = 0;
    q.push(finish);
    while (!q.empty()) {
        unsigned long long int current = q.front();
        q.pop();
        int currentDist = distances[current];
        if (currentDist == targetDistance) {
            statesAtTargetDistance.push_back(current);
            if (statesAtTargetDistance.size() > 1000) break;
            continue;
        }
        if (currentDist > targetDistance) continue;
        CalcVariants(current, neighbours);
        for (auto& neighbor : neighbours) {
            if (distances.find(neighbor) == distances.end()) {
                distances[neighbor] = currentDist + 1;
                q.push(neighbor);
            }
        }
    }
    if (!statesAtTargetDistance.empty()) {
        std::unordered_set<unsigned long long int> processed;
        std::uniform_int_distribution<unsigned long> dist(0, statesAtTargetDistance.size() - 1);
        unsigned long long int selected;
        for (int attempt = 0; attempt < statesAtTargetDistance.size(); ++attempt)
        {
            selected = statesAtTargetDistance[dist(rng)];
            if (processed.count(selected) && attempt != 0)
            {
                --attempt;
                continue;
            }
            std::vector<unsigned short> field;
            longToField(selected, field);
            std::vector<std::vector<unsigned short>> solutionPath = DoubleAStar(field);
            if (std::abs(static_cast<int>(solutionPath.size() - targetDistance)) <= 1) break;
            processed.insert(selected);
        }
        if (processed.size() == statesAtTargetDistance.size())
        {
            std::cout << "Не найдено состояний с длиной решения " << targetDistance << '\n';
        }
        else
        {
            std::cout << "Сгенерировано состояние с длиной решения " << targetDistance << '\n';
        }
        return selected;
    }
    std::cout << "Не найдено состояний с длиной решения " << targetDistance << '\n';
    return finish;
}
