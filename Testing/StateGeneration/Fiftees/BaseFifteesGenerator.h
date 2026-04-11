#ifndef UNIFIEDGRAPHALGORITHMS_BASEFIFTEESGENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_BASEFIFTEESGENERATOR_H

template<typename GraphType>
class IFifteesGeneratorType {
public:
    using State = typename GraphType::State;
    virtual ~IFifteesGeneratorType() = default;

    /// Генерирует состояние графа graph со сложностью complexity относительно goal
    virtual State generate(const GraphType& graph, State goal, int complexity) = 0;
};

#endif