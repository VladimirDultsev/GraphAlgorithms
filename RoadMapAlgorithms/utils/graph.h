#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_set>
#include <map>
#include <vector>
#include <utility>
#include <functional>
#include "../rapidjson/include/rapidjson/document.h"

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2;
    }
};

namespace Graph {
    // Глобальные структуры данных графа
    extern std::unordered_map<unsigned long long int, std::pair<double, double>> points;
    extern std::unordered_map<std::pair<double, double>, int, pair_hash> Indices;
    extern std::unordered_map<unsigned long long int, std::unordered_set<unsigned long long int>> adjList;
    extern std::unordered_map<std::pair<double, double>, 
                              std::unordered_set<std::pair<double, double>, pair_hash>, 
                              pair_hash> adjListCoords;
    extern std::vector<std::pair<double, double>> pts;
    extern unsigned long long int start, finish;
    
    // Функции работы с графом
    void buildGraph(rapidjson::Document& data, 
                   std::unordered_map<std::pair<double, double>, 
                   std::unordered_set<std::pair<double, double>, pair_hash>, 
                   pair_hash>& adj);
    void codePoints();
    double getLength(std::vector<unsigned long long int>& path);
    void printConnectivity();
    void scan();
    
    // Функции для алгоритмов
    double heuristic(unsigned long long int point, int AStarIndex);
    void setStartFinish(unsigned long long int s, unsigned long long int f);
}

#endif // GRAPH_H