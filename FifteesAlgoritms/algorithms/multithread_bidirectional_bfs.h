#ifndef MULTITHREAD_BIDIRECTIONAL_BFS_H
#define MULTITHREAD_BIDIRECTIONAL_BFS_H
#include <vector>

/// Модифицированная версия BFS для запуска из разных потоков
void ModifiedBFS(int BFSIndex);

/// Многопоточный 2BFS
std::vector<std::vector<unsigned short>> Multithread_2BFS(std::vector<unsigned short>& st);

#endif
