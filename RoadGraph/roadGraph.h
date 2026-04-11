#ifndef ROAD_GRAPH_H
#define ROAD_GRAPH_H

#include "../Core/Graph.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>

// Структура для хэширования пар
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2;
    }
};

class RoadGraph : public core::Graph<unsigned long long int> {
public:
    using State = unsigned long long int;

    // Конструктор: загружает граф из JSON-файла
    explicit RoadGraph(const std::string& filename);

    // Методы интерфейса Graph
    std::vector<State> getNeighbors(const State& state) const override;
    double heuristic(const State& from, const State& to) const override;
    bool isGoal(const State& current, const State& goal) const override;
    double edgeCost(const State& from, const State& to) const override;

    // Дополнительные методы
    // Получить индекс точки по координатам (поиск по точкам)
    State getIndexByCoords(double lon, double lat) const;
    // Получить координаты точки по индексу
    std::pair<double, double> getCoords(State idx) const;
    // Вычислить длину пути
    double getPathLength(const std::vector<State>& path) const;
    // Вывести информацию о графе
    void printInfo() const;
    // Вернуть координаты точек
    std::unordered_map<State, std::pair<double, double>> getPoints() const;

private:
    /// индекс -> (долгота, широта)
    std::unordered_map<State, std::pair<double, double>> points_;

    /// координаты -> индекс
    std::unordered_map<std::pair<double, double>, State, pair_hash> indices_;

    /// список смежности (индексы)
    std::unordered_map<State, std::unordered_set<State>> adjList_;

    /// список координат по порядку индексов
    std::vector<std::pair<double, double>> pts_;

    State start_, finish_;

    /// Строит список смежности из файла
    void buildFromJSON(const std::string& filename);
};

#endif