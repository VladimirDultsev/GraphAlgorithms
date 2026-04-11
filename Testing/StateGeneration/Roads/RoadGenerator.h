#ifndef UNIFIEDGRAPHALGORITHMS_ROADGENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_ROADGENERATOR_H

#include <random>
#include "../ITestGenerator.h"
#include "../../../RoadGraph/Utils/CalcDistance/distance.h"
#include "../../../RoadGraph/Utils/FindNearestPoint/LinearNearestPointFinder.h"

template<typename GraphType>
class RoadTestGenerator : public ITestGenerator<GraphType> {
    using State = typename GraphType::State;
public:
    RoadTestGenerator(const GraphType& graph,
                      double minLat, double maxLat,
                      double minLon, double maxLon,
                      double distanceThreshold = 2.0)
            : graph_(graph), minLat_(minLat), maxLat_(maxLat),
              minLon_(minLon), maxLon_(maxLon),
              distanceThreshold_(distanceThreshold),
              rng_(std::random_device{}()),
              latDist_(minLat, maxLat),
              lonDist_(minLon, maxLon) {

    }

    /// Генерируем точки в заданном прямоугольнике координат на заданном расстоянии
    std::pair<State, State> generate(int complexity) override {
        auto targetDist = static_cast<double>(complexity);
        while (true) {
            double lat1 = latDist_(rng_), lon1 = lonDist_(rng_);
            double lat2 = latDist_(rng_), lon2 = lonDist_(rng_);
            double dist = Distance::calcGPSDistance(lon1, lat1, lon2, lat2);
            if (std::abs(dist - targetDist) <= distanceThreshold_) {
                State s1 = findNearest(lat1, lon1);
                State s2 = findNearest(lat2, lon2);
                if (s1 != s2)
                    return {s1, s2};
            }
        }
    }

private:
    const GraphType& graph_;
    double minLat_, maxLat_, minLon_, maxLon_;
    double distanceThreshold_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> latDist_, lonDist_;
    LinearNearestPointFinder<RoadGraph> finder;

    State findNearest(double lat, double lon) const {
        return finder.findNearest(graph_, lat, lon);
    }
};

#endif

