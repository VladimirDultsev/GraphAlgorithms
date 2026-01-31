cmake_minimum_required(VERSION 3.10)
project(pathfinding)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES
        main.cpp
        utils/fiftees_generator.cpp
        utils/graph.cpp
        utils/heuristics.cpp
        algorithms/bfs.cpp
        algorithms/bidirectional_bfs.cpp
        algorithms/multithread_bidirectional_bfs.cpp
        algorithms/astar.cpp
        algorithms/bidirectional_astar.cpp
        algorithms/multithread_bidirectional_astar.cpp
        test_system.cpp
)

add_executable(fifteesSolver ${SOURCES})

find_package(Threads REQUIRED)
target_link_libraries(fifteesSolver Threads::Threads)