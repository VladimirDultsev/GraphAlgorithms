#include "roadGraph.h"
#include "Utils/CalcDistance/distance.h"
#include "Dependencies/rapidjson/include/rapidjson/document.h"
#include <fstream>
#include <iostream>
#include <limits>

RoadGraph::RoadGraph(const std::string& filename) {
    // Строим список смежности из переданного файла
    buildFromJSON(filename);
}

/// Строит список смежности из файла
void RoadGraph::buildFromJSON(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());

    rapidjson::Document doc;

    doc.Parse(content.c_str());
    if (doc.HasParseError()) {
        throw std::runtime_error("JSON parse error");
    }

    std::unordered_map<std::pair<double, double>,
            std::unordered_set<std::pair<double, double>, pair_hash>,
            pair_hash> adjCoords;

    if (!doc.HasMember("features") || !doc["features"].IsArray()) {
        throw std::runtime_error("Invalid JSON structure: missing features array");
    }

    const rapidjson::Value& features = doc["features"];

    for (rapidjson::SizeType i = 0; i < features.Size(); ++i) {
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

            double lon = point[0].GetDouble();
            double lat = point[1].GetDouble();
            double lastLon = lastPoint[0].GetDouble();
            double lastLat = lastPoint[1].GetDouble();

            adjCoords[{lon, lat}].insert({lastLon, lastLat});
            adjCoords[{lastLon, lastLat}].insert({lon, lat});
        }
    }

    // Кодируем вершины (задаём им индексы)
    State cnt = 0;
    for (const auto& pr : adjCoords) {
        points_[cnt] = pr.first;
        pts_.push_back(pr.first);
        indices_[pr.first] = cnt;
        ++cnt;
    }

    for (const auto& pr : adjCoords) {
        State from = indices_[pr.first];
        for (const auto& toCoord : pr.second) {
            State to = indices_[toCoord];
            adjList_[from].insert(to);
        }
    }
}

/// Возвращает список соседей вершины
std::vector<RoadGraph::State> RoadGraph::getNeighbors(const State& state) const {
    std::vector<State> result;
    auto it = adjList_.find(state);
    if (it != adjList_.end()) {
        result.reserve(it->second.size());
        for (State s : it->second) {
            result.push_back(s);
        }
    }
    return result;
}

/// Считает эвристику между from и to
double RoadGraph::heuristic(const State& from, const State& to) const {
    const auto& p1 = points_.at(from);
    const auto& p2 = points_.at(to);
    return Distance::calcGPSDistance(p1.first, p1.second, p2.first, p2.second);
}

/// Считает стоимость прохода по ребру между from и to (так как ребро прямое - это просто расстояние между ними)
double RoadGraph::edgeCost(const State& from, const State& to) const {
    auto it = adjList_.find(from);
    if (it != adjList_.end() && it->second.count(to)) {
        const auto& p1 = points_.at(from);
        const auto& p2 = points_.at(to);
        return Distance::calcGPSDistance(p1.first, p1.second, p2.first, p2.second);
    }
    return std::numeric_limits<double>::infinity();
}

/// Возвращает индекс вершины по её координатам
RoadGraph::State RoadGraph::getIndexByCoords(double lon, double lat) const {
    auto it = indices_.find({lon, lat});
    if (it != indices_.end()) {
        return it->second;
    }
    return static_cast<State>(-1);
}

/// Возвращает координаты вершины по её индексу
std::pair<double, double> RoadGraph::getCoords(State idx) const {
    return points_.at(idx);
}

/// Считает длину пути path
double RoadGraph::getPathLength(const std::vector<State>& path) const {
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        total += edgeCost(path[i-1], path[i]);
    }
    return total;
}

/// Возвращает все вершины графа
std::unordered_map<RoadGraph::State, std::pair<double, double>> RoadGraph::getPoints() const {
    return this->points_;
}

/// Выводит информацию о графе
void RoadGraph::printInfo() const {
    std::cout << "Number of vertices: " << points_.size() << std::endl;
    std::cout << "Number of edges: ";
    size_t edgeCount = 0;
    for (const auto& p : adjList_) {
        edgeCount += p.second.size();
    }
    std::cout << edgeCount / 2 << "\n";
    std::vector<int> degreeCnt(30, 0);
    for (const auto& p : adjList_) {
        size_t deg = p.second.size();
        if (deg < degreeCnt.size()) {
            ++degreeCnt[deg];
        }
    }

    std::cout << "Degrees:" << std::endl;
    for (size_t i = 0; i < degreeCnt.size(); ++i) {
        if (degreeCnt[i] > 0) {
            std::cout << "  Degree " << i << ": " << degreeCnt[i] << std::endl;
        }
    }
}

/// Проверяет является ли вершина целевой
bool RoadGraph::isGoal(const State& current, const State& goal) const {
    return current == goal;
}