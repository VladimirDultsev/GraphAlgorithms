#ifndef TEST_SYSTEM_H
#define TEST_SYSTEM_H

#include <iostream>
#include <utility>
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <map>
#include <random>
#include <memory>
#include <queue>
#include <unordered_map>
#include <cmath>

#include "../RoadGraph/Utils/CalcDistance/distance.h"
#include "StateGeneration/ITestGenerator.h"

template<typename GraphType>
class TestSystem {
public:
    using State = typename GraphType::State;

    TestSystem(const GraphType& graph,
               std::unique_ptr<ITestGenerator<GraphType>> generator,
               std::vector<std::function<std::vector<State>(State, State)>> algorithms,
               std::vector<std::string> names,
               double timeLimit = 60.0)
            : graph_(graph), generator_(std::move(generator)),
              algorithms_(algorithms), names_(std::move(names)), timeLimit_(timeLimit) {}

    void run(const std::vector<int>& complexities, int testsPerComplexity) {
        std::vector<std::map<int, double>> results(algorithms_.size());
        std::vector<std::map<int, int>> successCount(algorithms_.size());

        for (int complexity : complexities) {
            std::cout << "Complexity: " << complexity << " \n\n";
            for (int test = 0; test < testsPerComplexity; ++test) {
                std::cout << "Generating";
                auto [start, goal] = generator_->generate(complexity);
                std::cout << " finished\n\n";

                for (int algInd = 0; algInd < algorithms_.size(); ++algInd) {
                    std::cout << "Running " << names_[algInd] << "...\n";

                    auto startTime = std::chrono::steady_clock::now();
                    auto path = algorithms_[algInd](start, goal);
                    auto endTime = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(endTime - startTime).count();

                    results[algInd][complexity] += elapsed;
                    std::cout << names_[algInd] << " finished " << (path.empty() ? "with error": "successfully") << "\n\n";

                    if (!path.empty()) {
                        successCount[algInd][complexity]++;
                    } else {
                        std::cout << "  " << names_[algInd] << " error for test " << test << "\n";
                    }
                }
            }
        }

        std::cout << "\nRESULTS\n";
        for (int algInd = 0; algInd < algorithms_.size(); ++algInd) {
            std::cout << "\n" << names_[algInd] << ":\n";
            for (int complexity : complexities) {
                double totalTime = results[algInd][complexity];
                int success = successCount[algInd][complexity];
                double avgTime = totalTime / testsPerComplexity;
                std::cout << "  Complexity " << complexity << ": success " << success << "/" << testsPerComplexity
                          << ", avg time " << avgTime << " s\n";
            }
        }
    }

private:
    const GraphType& graph_;
    std::unique_ptr<ITestGenerator<GraphType>> generator_;
    std::vector<std::function<std::vector<State>(State, State)>> algorithms_;
    std::vector<std::string> names_;
    double timeLimit_;
};

template<typename GraphType, typename SolverType>
std::function<std::vector<typename GraphType::State>(typename GraphType::State, typename GraphType::State)>
makeSolver(const GraphType& graph, double timeLimit) {
    return [&graph, timeLimit](auto start, auto goal) {
        SolverType solver(graph);
        solver.setTimeLimit(timeLimit);
        return solver.solve(start, goal);
    };
}

#endif