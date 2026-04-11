#ifndef ROAD_GRAPH_LINEAR_NEAREST_POINT_FINDER_H
#define ROAD_GRAPH_LINEAR_NEAREST_POINT_FINDER_H

#include "INearestPointFinder.h"
#include "../CalcDistance/distance.h"
#include <limits>

template<typename GraphType>
class LinearNearestPointFinder : public INearestPointFinder<GraphType> {
public:
    using State = typename GraphType::State;

    /// Линейный поиск ближайшей точки графа
    State findNearest(const GraphType& graph, double longitude, double latitude) const override {
        double minDist = std::numeric_limits<double>::max();
        State nearestPoint = 0;
        for (const auto& [id, coords] : graph.getPoints()) {
            double dist = Distance::calcGPSDistance(longitude, latitude, coords.second, coords.first);
            if (dist < minDist) {
                minDist = dist;
                nearestPoint = id;
            }
        }
        return nearestPoint;
    }
};

#endif