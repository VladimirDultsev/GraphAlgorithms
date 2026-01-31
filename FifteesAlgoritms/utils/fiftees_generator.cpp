#include "fiftees_generator.h"
#include "../algorithms/bidirectional_astar.h"
#include <iostream>
#include <queue>
#include <random>
#include <unordered_map>
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
        std::uniform_int_distribution<unsigned long> dist(0, statesAtTargetDistance.size() - 1);
        unsigned long long int selected = statesAtTargetDistance[dist(rng)];
        std::vector<unsigned short> field;
        longToField(selected, field);
        std::vector<std::vector<unsigned short>> solutionPath = DoubleAStar(field);
        std::cout << "Сгенерировано состояние с длиной решения: " << solutionPath.size() << '\n';
        return selected;
    }
    std::cout << "Не найдено состояний с расстоянием " << targetDistance << '\n';
    return finish;
}
