#include "Testing/TestSystem.h"
#include "RoadGraph/roadGraph.h"
#include "FifteesGraph/fifteesGraph.h"
#include "Algorithms/AStar/AStar.h"
#include "Algorithms/BidirectionalAStar/BidirectionalAStar.h"
#include "Algorithms/MultithreadBidirectionalAStar/MultithreadBidirectionalAStar.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/BidirectionalDijkstra/BidirectionalDijkstra.h"
#include "Algorithms/MultithreadBidirectionalDijkstra/MultithreadBidirectionalDijkstra.h"
#include "Testing/StateGeneration/Roads/RoadGenerator.h"
#include "Testing/StateGeneration/Fiftees/FifteesTestGenerator.h"

int main() {
    RoadGraph roadGraph("RoadGraph/Graphs/moscow_roads.geojson");
    auto roadGen = std::make_unique<RoadTestGenerator<RoadGraph>>(
            roadGraph, 55.55, 55.90, 37.30, 37.85, 2.0);

    std::vector<std::function<std::vector<RoadGraph::State>(RoadGraph::State, RoadGraph::State)>> roadAlgos;
    std::vector<std::string> roadNames;

    roadAlgos.push_back(makeSolver<RoadGraph, AStar<RoadGraph>>(roadGraph, 60.0));
    roadNames.push_back("AStar");

    roadAlgos.push_back(makeSolver<RoadGraph, BidirectionalAStar<RoadGraph>>(roadGraph, 60.0));
    roadNames.push_back("BidirectionalAStar");

    roadAlgos.push_back(makeSolver<RoadGraph, MultithreadBidirectionalAStar<RoadGraph>>(roadGraph, 60.0));
    roadNames.push_back("MultithreadBidirectionalAStar");

    TestSystem<RoadGraph> roadTester(roadGraph, std::move(roadGen), roadAlgos, roadNames);
    std::vector<int> distances = {10, 20, 30, 40, 50}; // километры
    roadTester.run(distances, 5);

    uint64_t goal = 1311768467463790320;
    FifteesGraph puzzleGraph(goal);
    auto fifteesGen = createHybridFifteesGenerator(puzzleGraph, goal, 0.8);

    std::vector<std::function<std::vector<uint64_t>(uint64_t, uint64_t)>> fifteesAlgos;
    std::vector<std::string> fifteesNames;

    fifteesAlgos.push_back(makeSolver<FifteesGraph, AStar<FifteesGraph>>(puzzleGraph, 30.0));
    fifteesNames.push_back("AStar");

    fifteesAlgos.push_back(makeSolver<FifteesGraph, BidirectionalAStar<FifteesGraph>>(puzzleGraph, 30.0));
    fifteesNames.push_back("BidirectionalAStar");

    fifteesAlgos.push_back(makeSolver<FifteesGraph, MultithreadBidirectionalAStar<FifteesGraph>>(puzzleGraph, 30.0));
    fifteesNames.push_back("MultithreadBidirectionalAStar");

    TestSystem<FifteesGraph> puzzleTester(puzzleGraph, std::move(fifteesGen), fifteesAlgos, fifteesNames);
    std::vector<int> moves = {10, 20, 30, 40, 50};
    puzzleTester.run(moves, 5);

    return 0;
}