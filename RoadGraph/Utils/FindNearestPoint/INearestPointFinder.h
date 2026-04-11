#ifndef ROAD_GRAPH_INEAREST_POINT_FINDER_H
#define ROAD_GRAPH_INEAREST_POINT_FINDER_H

#include <utility>

template<typename GraphType>
class INearestPointFinder {
public:
    using State = typename GraphType::State;
    virtual ~INearestPointFinder() = default;

    /// Возвращает индекс ближайшей точки графа к заданным координатам
    virtual State findNearest(const GraphType& graph, double longitude, double latitude) const = 0;
};

#endif