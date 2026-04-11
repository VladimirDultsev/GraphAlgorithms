#ifndef ROAD_GRAPH_SPATIAL_NEAREST_POINT_FINDER_H
#define ROAD_GRAPH_SPATIAL_NEAREST_POINT_FINDER_H

#include "INearestPointFinder.h"
#include "../CalcDistance/distance.h"
#include "LinearNearestPointFinder.h"
#include <map>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

template<typename GraphType>
class SpatialNearestPointFinder : public INearestPointFinder<GraphType> {
public:
    using State = typename GraphType::State;

    // sideSize – количество ячеек сетки
    explicit SpatialNearestPointFinder(int sideSize = 100) : sideSize_(sideSize) {
        splitDone_ = false;
    }

    /// Быстрый алгоритм поиска ближайшей точки
    State findNearest(const GraphType& graph, double longitude, double latitude) const override {
        // Если разбиение ещё не выполнено – выполняем его
        if (!splitDone_) {
            const_cast<SpatialNearestPointFinder*>(this)->buildSpatialIndex(graph);
        }

        // Если после разбиения структура пуста – используем линейный поиск
        if (splittedGraphs_.empty()) {
            return linearFallback(graph, longitude, latitude);
        }

        double minDist = std::numeric_limits<double>::max();
        std::pair<double, double> nearestPoint;

        // Определяем широтный блок
        auto itLat = splittedGraphs_.lower_bound(latitude);
        if (itLat == splittedGraphs_.end()) itLat = std::prev(splittedGraphs_.end());
        std::vector<decltype(itLat)> latGroups = {itLat};
        if (itLat != splittedGraphs_.begin()) latGroups.push_back(std::prev(itLat));
        auto nextLat = std::next(itLat);
        if (nextLat != splittedGraphs_.end()) latGroups.push_back(nextLat);

        for (auto latGrp : latGroups) {
            const auto& lonDict = latGrp->second;
            if (lonDict.empty()) continue;

            auto itLon = lonDict.lower_bound(longitude);
            if (itLon == lonDict.end()) itLon = std::prev(lonDict.end());
            std::vector<decltype(itLon)> lonGroups = {itLon};
            if (itLon != lonDict.begin()) lonGroups.push_back(std::prev(itLon));
            auto nextLon = std::next(itLon);
            if (nextLon != lonDict.end()) lonGroups.push_back(nextLon);

            for (auto lonGrp : lonGroups) {
                for (const auto& point : lonGrp->second) {
                    double dist = Distance::calcGPSDistance(longitude, latitude, point.second, point.first);
                    if (dist < minDist) {
                        minDist = dist;
                        nearestPoint = point;
                    }
                }
            }
        }

        if (minDist == std::numeric_limits<double>::max()) {
            return linearFallback(graph, longitude, latitude);
        }

        // Преобразуем координаты в индекс (State)
        return graph.getIndexByCoords(nearestPoint.first, nearestPoint.second);
    }

private:
    int sideSize_;
    mutable bool splitDone_ = false;
    /// Структура: широта -> (долгота -> список точек)
    mutable std::map<double, std::map<double, std::vector<std::pair<double, double>>>> splittedGraphs_;

    /// Предпосчёт блоков разделения точек
    void buildSpatialIndex(const GraphType& graph) {
        // Получаем все точки графа
        auto points = graph.getPoints();
        std::vector<std::pair<double, double>> pts;
        pts.reserve(points.size());
        for (const auto& [id, coords] : points) {
            pts.push_back(coords);
        }

        // Сортировка по долготе (x)
        std::sort(pts.begin(), pts.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        int total = pts.size();
        int elementsInContainer = std::max(1, total / static_cast<int>(std::sqrt(sideSize_)));

        // Первый уровень разбиения по долготе
        std::map<double, std::vector<std::pair<double, double>>> tempSplit;
        std::vector<std::pair<double, double>> buffer;
        int cnt = 0;
        std::pair<double, double> lastPoint;

        for (const auto& point : pts) {
            if (cnt != 0 && cnt % elementsInContainer == 0 && cnt < sideSize_ * elementsInContainer) {
                tempSplit[lastPoint.first] = buffer;
                buffer.clear();
            }
            buffer.push_back(point);
            lastPoint = point;
            ++cnt;
        }
        if (!buffer.empty()) {
            if (tempSplit.empty()) {
                tempSplit[lastPoint.first] = buffer;
            } else {
                auto iter = std::prev(tempSplit.end());
                iter->second.insert(iter->second.end(), buffer.begin(), buffer.end());
            }
        }

        // Второй уровень разбиения по широте
        int secondLevelSize = std::max(1, elementsInContainer / static_cast<int>(std::sqrt(sideSize_)));

        for (auto& [lonKey, pointsInLonBlock] : tempSplit) {
            // Сортируем точки блока по широте (y)
            std::sort(pointsInLonBlock.begin(), pointsInLonBlock.end(),
                      [](const auto& a, const auto& b) { return a.second < b.second; });

            std::map<double, std::vector<std::pair<double, double>>> latDict;
            buffer.clear();
            cnt = 0;

            for (const auto& point : pointsInLonBlock) {
                if (cnt != 0 && cnt % secondLevelSize == 0 && cnt < sideSize_ * secondLevelSize) {
                    latDict[lastPoint.second] = buffer;
                    buffer.clear();
                }
                buffer.push_back(point);
                lastPoint = point;
                ++cnt;
            }
            if (!buffer.empty()) {
                if (latDict.empty()) {
                    latDict[lastPoint.second] = buffer;
                } else {
                    auto iter = std::prev(latDict.end());
                    iter->second.insert(iter->second.end(), buffer.begin(), buffer.end());
                }
            }
            for (auto& [latKey, vec] : latDict) {
                splittedGraphs_[latKey][lonKey] = std::move(vec);
            }
        }
        splitDone_ = true;
    }

    /// Если быстрый поиск не сработал - используется линейный
    State linearFallback(const GraphType& graph, double longitude, double latitude) const {
        LinearNearestPointFinder<GraphType> linearFinder;
        return linearFinder.findNearest(graph, longitude, latitude);
    }
};

#endif