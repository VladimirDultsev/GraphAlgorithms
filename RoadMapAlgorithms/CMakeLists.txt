cmake_minimum_required(VERSION 3.10)
project(pathfinding)

set(CMAKE_CXX_STANDARD 17)

include_directories(../rapidjson/include)

set(SOURCES
        main.cpp
        utils/distance.cpp
        utils/graph.cpp
        algorithms/astar.cpp
        algorithms/dijkstra.cpp
        algorithms/bidirectional_astar.cpp
        algorithms/multithread_bidirectional_astar.cpp
        test_system.cpp
)

add_executable(pathfinding ${SOURCES})

find_package(Threads REQUIRED)
target_link_libraries(pathfinding Threads::Threads)