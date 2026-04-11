cmake_minimum_required(VERSION 3.10)
project(UnifiedGraphAlgorithms VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
        main.cpp
        FifteesGraph/fifteesGraph.cpp
        RoadGraph/Utils/CalcDistance/distance.cpp
        RoadGraph/roadGraph.cpp
)

set(HEADERS
        Core/Graph.h
        Core/Solver.h
        Testing/TestSystem.h
        FifteesGraph/fifteesGraph.h
        Algorithms/BFS/BFS.h
        Algorithms/Dijkstra/Dijkstra.h
        Algorithms/AStar/AStar.h
        Algorithms/BidirectionalDijkstra/BidirectionalDijkstra.h
        Algorithms/BidirectionalAStar/BidirectionalAStar.h
        Algorithms/MultithreadBidirectionalDijkstra/MultithreadBidirectionalDijkstra.h
        Algorithms/MultithreadBidirectionalAStar/MultithreadBidirectionalAStar.h
        Algorithms/BidirectionalBFS/BidirectionalBFS.h
        Algorithms/MultithreadBidirectionalBFS/MultithreadBidirectionalBFS.h
        RoadGraph/roadGraph.h
        RoadGraph/Utils/CalcDistance/distance.h
        Testing/StateGeneration/ITestGenerator.h
        Testing/StateGeneration/Fiftees/BaseFifteesGenerator.h
        Testing/StateGeneration/Fiftees/HybridGenerator.h
        Testing/StateGeneration/Fiftees/BFSBasedGenerator.h
        Testing/StateGeneration/Fiftees/RandomWalkGenerator.h
        Testing/StateGeneration/Roads/RoadGenerator.h
        Testing/StateGeneration/Fiftees/FifteesTestGenerator.h
        RoadGraph/Utils/FindNearestPoint/INearestPointFinder.h
        RoadGraph/Utils/FindNearestPoint/LinearNearestPointFinder.h
        RoadGraph/Utils/FindNearestPoint/SpatialFindNearestPoint.h
)

add_executable(new_target ${SOURCES} ${HEADERS})

target_include_directories(new_target PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/RoadGraph/Dependencies/rapidjson/include
)

find_package(Threads REQUIRED)
target_link_libraries(new_target Threads::Threads)