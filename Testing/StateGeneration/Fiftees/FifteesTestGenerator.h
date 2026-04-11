#ifndef UNIFIEDGRAPHALGORITHMS_FIFTEESTESTGENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_FIFTEESTESTGENERATOR_H

#include <memory>
#include "../ITestGenerator.h"
#include "BaseFifteesGenerator.h"
#include "BFSBasedGenerator.h"
#include "HybridGenerator.h"
#include "RandomWalkGenerator.h"

// Класс-адаптер (реализует ITestGenerator, используя любой из IFifteesGeneratorType)
template<typename GraphType>
class FifteesTestGenerator : public ITestGenerator<GraphType> {
public:
    using State = typename GraphType::State;

    FifteesTestGenerator(const GraphType& graph, State goal,
                         std::unique_ptr<IFifteesGeneratorType<GraphType>> strategy)
            : graph_(graph), goal_(goal), strategy_(std::move(strategy)) {}

    std::pair<State, State> generate(int complexity) override {
        State start = strategy_->generate(graph_, goal_, complexity);
        return {start, goal_};
    }

private:
    const GraphType& graph_;
    State goal_;
    std::unique_ptr<IFifteesGeneratorType<GraphType>> strategy_;
};

template<typename GraphType>
std::unique_ptr<ITestGenerator<GraphType>> createBFSBasedFifteesGenerator(
        const GraphType& graph, typename GraphType::State goal) {
    auto strategy = std::make_unique<BFSBased<GraphType>>();
    return std::make_unique<FifteesTestGenerator<GraphType>>(graph, goal, std::move(strategy));
}

template<typename GraphType>
std::unique_ptr<ITestGenerator<GraphType>> createRandomWalkFifteesGenerator(
        const GraphType& graph, typename GraphType::State goal) {
    auto strategy = std::make_unique<RandomWalk<GraphType>>();
    return std::make_unique<FifteesTestGenerator<GraphType>>(graph, goal, std::move(strategy));
}

template<typename GraphType>
std::unique_ptr<ITestGenerator<GraphType>> createHybridFifteesGenerator(
        const GraphType& graph, typename GraphType::State goal, double greedyProb = 0.7) {
    auto strategy = std::make_unique<HybridStrategy<GraphType>>(greedyProb);
    return std::make_unique<FifteesTestGenerator<GraphType>>(graph, goal, std::move(strategy));
}

#endif