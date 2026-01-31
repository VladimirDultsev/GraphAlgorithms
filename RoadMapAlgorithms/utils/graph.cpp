#include "graph.h"
#include "distance.h"
#include "../rapidjson/include/rapidjson/document.h"
#include <iostream>

namespace Graph {
    // Определение глобальных переменных
    std::unordered_map<unsigned long long int, std::pair<double, double>> points;
    std::unordered_map<std::pair<double, double>, int, pair_hash> Indices;
    std::unordered_map<unsigned long long int, std::unordered_set<unsigned long long int>> adjList;
    std::unordered_map<std::pair<double, double>, 
                       std::unordered_set<std::pair<double, double>, pair_hash>, 
                       pair_hash> adjListCoords;
    std::vector<std::pair<double, double>> pts;
    unsigned long long int start, finish;
    
    /// Функция построения графа из JSON-документа
    void buildGraph(rapidjson::Document& data, std::unordered_map<std::pair<double, double>,
                   std::unordered_set<std::pair<double, double>, pair_hash>, 
                   pair_hash>& adj) {
        if (!data.HasMember("features") || !data["features"].IsArray()) {
            std::cerr << "Invalid JSON structure: missing features array" << std::endl;
            return;
        }
        const rapidjson::Value& features = data["features"];
        for (unsigned long long int i = 0; i < features.Size(); ++i) {
            const rapidjson::Value& feature = features[i];
            if (!feature.IsObject() ||
                !feature.HasMember("geometry") ||
                !feature["geometry"].IsObject() ||
                !feature["geometry"].HasMember("coordinates") ||
                !feature["geometry"]["coordinates"].IsArray()) {
                continue;
            }
            const rapidjson::Value& coordinates = feature["geometry"]["coordinates"];
            for (rapidjson::SizeType y = 1; y < coordinates.Size(); ++y) {
                const rapidjson::Value& point = coordinates[y];
                const rapidjson::Value& lastPoint = coordinates[y - 1];
                if (!point.IsArray() || point.Size() < 2 ||
                    !point[0].IsNumber() || !point[1].IsNumber()) {
                    continue;
                }
                adj[std::make_pair(point[0].GetDouble(), point[1].GetDouble())]
                    .insert(std::make_pair(lastPoint[0].GetDouble(), lastPoint[1].GetDouble()));
                adj[std::make_pair(lastPoint[0].GetDouble(), lastPoint[1].GetDouble())]
                    .insert(std::make_pair(point[0].GetDouble(), point[1].GetDouble()));
            }
        }
    }
    
    /// Функция, задающая номера точкам
    void codePoints(){
        unsigned long long int cnt = 0;
        for(auto pr: adjListCoords){
            points[cnt] = pr.first;
            pts.push_back(pr.first);
            Indices[pr.first] = cnt;
            ++cnt;
        }
        for(auto pr: adjListCoords){
            for(auto pr1: pr.second){
                adjList[Indices[pr.first]].insert(Indices[pr1]);
            }
        }
    }
    
    /// Эвристическая функция для A*
    double heuristic(const unsigned long long int point, const int AStarIndex){
        // В текущей версии Двухпоточный алгоритм Дейкстры и 2Дейкстра - это Двухпоточный A* и 2A*, но эвристическая функция возвращает 0

        if(start == 0 && finish == 0) return 0; // Защита от неинициализированных точек
        
        if(AStarIndex){ // Если нас вызывает обратный A*, возвращаем расстояние от текущей точки до старта
            return Distance::calcGPSDistance(
                points[start].first, points[start].second,
                points[point].first, points[point].second);
        }
        // Если нас вызывает прямой A*, возвращаем расстояние от текущей точки до финиша
        return Distance::calcGPSDistance(
            points[finish].first, points[finish].second,
            points[point].first, points[point].second);
    }
    
    /// Установка стартовой и конечной точек
    void setStartFinish(const unsigned long long int s, const unsigned long long int f){
        start = s;
        finish = f;
    }
    
    /// Функция, определяющая суммарную длину пути
    double getLength(const std::vector<unsigned long long int>& path){
        double sum = 0;
        for(int i = 1; i < path.size(); ++i){
            sum += Distance::calcGPSDistance(
                points[path[i - 1]].first, points[path[i - 1]].second,
                points[path[i]].first, points[path[i]].second);
        }
        return sum;
    }
    
    /// Функция, выводящая количество точек с данным количеством соседей (с помощью неё можно оценить связность графа)
    void printConnectivity(){
        std::vector<int> sums(30);
        for(auto pair:adjList){
            ++sums[pair.second.size()];
        }
        for(int cnt: sums){
            std::cout << cnt << '\n';
        }
    }
    
    /// Считает минимальные и максимальные координаты точек графа
    void scan(){
        double minFst = std::numeric_limits<double>::max();
        double maxFst = std::numeric_limits<double>::min();
        double minSnd = std::numeric_limits<double>::max();
        double maxSnd = std::numeric_limits<double>::min();
        for(auto pr: points){
            if(pr.second.first < minFst){
                minFst = pr.second.first;
            }
            if(pr.second.first > maxFst){
                maxFst = pr.second.first;
            }
            if(pr.second.second < minSnd){
                minSnd = pr.second.second;
            }
            if(pr.second.second > maxSnd){
                maxSnd = pr.second.second;
            }
        }
        std::cout << "Min longitude: " << minFst << ", Max longitude: " << maxFst << std::endl;
        std::cout << "Min latitude: " << minSnd << ", Max latitude: " << maxSnd << std::endl;
    }
}