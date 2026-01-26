#ifndef MULTITHREAD_BIDIRECTIONAL_H
#define MULTITHREAD_BIDIRECTIONAL_H

#include <vector>

/// Многопоточный двунаправленный A*
std::vector<unsigned long long int> MultithreadDoubleAStar();
/// Многопоточная модификация A*
void ModifiedAStar(int AStarIndex);
void ClearMultithreadDoubleAStarData();

#endif