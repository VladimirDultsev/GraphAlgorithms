#include "multithread_bidirectional_astar.h"
#include "../utils/graph.h"
#include "../utils/distance.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>

extern bool isDejkstra; // Флаг для определения, используем ли мы эвристику

namespace {
    std::map<double, std::unordered_set<unsigned long long int>> dict[2];
    std::unordered_map<unsigned long long int,
                      std::tuple<double, double, unsigned long long int, bool>> pars[2];
    std::mutex mtx;
    std::unordered_set<unsigned long long int> used;
    std::atomic<bool> needToFinish{false};
    std::mutex cv_mtx;
    std::condition_variable cv;
    unsigned long long int buffPoint;
}
void ClearMultithreadDoubleAStarData() {
    dict[0].clear();
    dict[1].clear();
    pars[0].clear();
    pars[1].clear();
    used.clear();
    needToFinish.store(false);
    buffPoint = 0;
}
using namespace Graph;
/// Многопоточная модификация A*
void ModifiedAStar(int AStarIndex)
{
    unsigned long long int point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;

    // Добавляем информацию о стартовой вершине
    if(!AStarIndex){
        pars[AStarIndex][start] = std::make_tuple(Graph::heuristic(start, AStarIndex), 0, start, false);
        dict[AStarIndex][std::get<0>(pars[AStarIndex][start])].insert(start);
    } else {
        pars[AStarIndex][finish] = std::make_tuple(Graph::heuristic(finish, AStarIndex), 0, finish, false);
        dict[AStarIndex][std::get<0>(pars[AStarIndex][finish])].insert(finish);
    }

    while (!dict[AStarIndex].empty() && !needToFinish)
    {
        point = *(dict[AStarIndex].begin()->second.begin());
        
        // Чтобы проверить не обработана ли данная точка противонаправленным
        // алгоритмом, нужен mutex, так как структура данных used используется в двух потоках
        mtx.lock();
        if ((!AStarIndex && point == finish) || (AStarIndex && point == start) || used.count(point)) {
            buffPoint = point;
            needToFinish = true;
            mtx.unlock();
            cv.notify_all();
            return;
        }
        used.insert(point);
        mtx.unlock();

        for (unsigned long long int neighbour : Graph::adjList[point]) // Перебираем соседей точки
        {
            newNeighbourDist = std::get<1>(pars[AStarIndex][point]) + // Расстояние, за которое мы дошли до этого соседа
                             Distance::calcGPSDistance(
                                 Graph::points[point].first, Graph::points[point].second,
                                 Graph::points[neighbour].first, Graph::points[neighbour].second);
            
            if (!pars[AStarIndex].count(neighbour)) // Если до этого мы не встречали нашего соседа
            {
                heuristic = isDejkstra ? 0 : Graph::heuristic(neighbour, AStarIndex); // Считаем эвристику до него
                pars[AStarIndex][neighbour] = std::make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic; // В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
            else if (newNeighbourDist < std::get<1>(pars[AStarIndex][neighbour])) // Если найден более короткий путь до соседа
            {
                oldDist = std::get<0>(pars[AStarIndex][neighbour]) + // Считаем приоритет, под которым сосед был положен в словарь раньше
                         std::get<1>(pars[AStarIndex][neighbour]);
                dict[AStarIndex][oldDist].erase(neighbour); // Удаляем соседа из словаря
                if (dict[AStarIndex][oldDist].empty()) { // Если в словаре больше нет точек с таким приоритетом
                    dict[AStarIndex].erase(oldDist); // Удаляем этот ключ из словаря
                }
                std::get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist; // Обновляем расстояние до соседа
                std::get<2>(pars[AStarIndex][neighbour]) = point; // Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + std::get<0>(pars[AStarIndex][neighbour]); // В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }
        
        std::get<3>(pars[AStarIndex][point]) = true; // Отмечаем нашу точку удаленной
        oldDist = std::get<0>(pars[AStarIndex][point]) + // Считаем приоритет, под которым наша точка лежала в словаре
                 std::get<1>(pars[AStarIndex][point]);
        dict[AStarIndex][oldDist].erase(point); // Удаляем точку из словаря
        if (dict[AStarIndex][oldDist].empty()) { // Если в словаре больше нет точек с таким приоритетом
            dict[AStarIndex].erase(oldDist); // Удаляем этот ключ из словаря
        }
    }
}

/// Многопоточный двунаправленный A*
std::vector<unsigned long long int> MultithreadDoubleAStar()
{
    used.clear();
    needToFinish = false;
    
    // Запускаем два потока с прямым и обратным обходами
    std::thread ModifiedAStar1(ModifiedAStar, 0);
    std::thread ModifiedAStar2(ModifiedAStar, 1);
    {
        // Ожидаем сигнала о завершении от одного из потоков
        std::unique_lock<std::mutex> lk(cv_mtx);
        cv.wait(lk, []{ return needToFinish.load(); });
    }
    
    // Ждём завершения потоков
    if(ModifiedAStar1.joinable()){
        ModifiedAStar1.join();
    }
    if(ModifiedAStar2.joinable()){
        ModifiedAStar2.join();
    }

    // Восстановление пути
    std::vector<unsigned long long int> path;
    for(unsigned long long int pt = buffPoint; pt != start; pt = std::get<2>(pars[0][pt])){ // Прыгаем по соседям из pars прямого A*-а пока не дойдем до старта
        path.push_back(pt);
    }
    path.push_back(start); // Добавляем старт в путь
    std::reverse(path.begin(), path.end()); // Переворачиваем путь, потому что мы шли в обратном порядке
    
    for(unsigned long long int pt = std::get<2>(pars[1][buffPoint]); pt != finish; pt = std::get<2>(pars[1][pt])){ // Прыгаем по соседям из pars обратного A*-а пока не дойдем до финиша
        path.push_back(pt);
    }
    path.push_back(finish); // Добавляем финиш в путь
    return path;
}