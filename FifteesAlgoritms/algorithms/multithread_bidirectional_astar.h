#ifndef MULTITHREAD_BIDIRECTIONAL_ASTAR_H
#define MULTITHREAD_BIDIRECTIONAL_ASTAR_H
#include <vector>

/// Модифицированная версия A* для запуска из разных потоков
void ModifiedAStar(int AStarIndex);

/// Многопоточный 2A*
std::vector<std::vector<unsigned short>> Multithread_2AStar(std::vector<unsigned short>& st);

#endif
