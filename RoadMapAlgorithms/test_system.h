#ifndef TEST_SYSTEM_H
#define TEST_SYSTEM_H

namespace TestSystem {
    /// Тестирующая система для алгоритмов
    void Test(int amountOfStages, int threshold, int testsInStage, int step);
}

/// Флаг для определения, используем ли мы эвристику
extern bool isDejkstra;

#endif