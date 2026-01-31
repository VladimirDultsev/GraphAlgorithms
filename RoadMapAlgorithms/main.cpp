#include <iostream>
#include "algorithms/astar.h"
#include "algorithms/dijkstra.h"
#include "algorithms/bidirectional_astar.h"
#include "algorithms/multithread_bidirectional_astar.h"
#include "utils/graph.h"
#include "test_system.h"
#include "rapidjson/include/rapidjson/document.h"
#include "rapidjson/include/rapidjson/istreamwrapper.h"
#include <fstream>

int main()
{
    std::ifstream file("../Graphs/moscow_roads.geojson");
    rapidjson::IStreamWrapper isw(file);
    rapidjson::Document doc;
    doc.ParseStream(isw);
    
    Graph::buildGraph(doc, Graph::adjListCoords);
    Graph::codePoints();
    
    TestSystem::Test(68, 500, 100, 1000);
    return 0;
}