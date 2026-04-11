#ifndef UNIFIEDGRAPHALGORITHMS_ITESTGENERATOR_H
#define UNIFIEDGRAPHALGORITHMS_ITESTGENERATOR_H

#include <utility>

template<typename GraphType>
class ITestGenerator {
public:
    using State = typename GraphType::State;
    virtual ~ITestGenerator() = default;
    virtual std::pair<State, State> generate(int complexity) = 0;
};

#endif
