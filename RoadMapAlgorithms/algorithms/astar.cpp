#include "astar.h"
#include "../utils/graph.h"
#include "../utils/distance.h"
#include <algorithm>

extern bool isDejkstra; // Определено в test_system.cpp

using namespace Graph;
/// A*
std::vector<unsigned long long int> AStar()
{
    int AStarIndex = 0; // Так как этот A* однонаправленный, чётко фиксируем его индекс
    unsigned long long int point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;
    std::map<double, std::unordered_set<unsigned long long int>> dict;
    std::unordered_map<unsigned long long int, std::tuple<double, double, unsigned long long int, bool>> pars;

    pars[start] = std::make_tuple(Graph::heuristic(start, AStarIndex), 0, start, false); // Добавляем информацию о стартовой вершине
    dict[std::get<0>(pars[start])].insert(start); // Добавляем старт в словарь
    
    while (!dict.empty()) // Алгоритм работает пока в словаре есть точки
    {
        point = *(dict.begin()->second.begin()); // Текущая точка - случайная из наименьших по расстоянию точек словаря
        
        if (point == finish) // Дошли до финиша - завершаемся
        {
            //cout << "solved\n";
            break;
        }
        
        for (unsigned long long int neighbour : Graph::adjList[point]) // Перебираем соседей точки
        {
            newNeighbourDist = std::get<1>(pars[point]) + // Расстояние, за которое мы дошли до этого соседа
                             Distance::calcGPSDistance(
                                 Graph::points[point].first, Graph::points[point].second,
                                 Graph::points[neighbour].first, Graph::points[neighbour].second);
            
            if (!pars.count(neighbour)) // Если до этого мы не встречали нашего соседа
            {
                heuristic = isDejkstra ? 0 : Graph::heuristic(neighbour, AStarIndex); // Считаем эвристику до него
                pars[neighbour] = std::make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic; // В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[finalDist].insert(neighbour);
            }
            else if (newNeighbourDist < std::get<1>(pars[neighbour])) // Если найден более короткий путь до соседа
            {
                oldDist = std::get<0>(pars[neighbour]) + std::get<1>(pars[neighbour]); // Считаем приоритет, под которым сосед был положен в словарь раньше
                dict[oldDist].erase(neighbour); // Удаляем соседа из словаря
                if (dict[oldDist].empty()) { // Если в словаре больше нет точек с таким приоритетом
                    dict.erase(oldDist); // Удаляем этот ключ из словаря
                }
                std::get<1>(pars[neighbour]) = newNeighbourDist; // Обновляем расстояние до соседа
                std::get<2>(pars[neighbour]) = point; // Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + std::get<0>(pars[neighbour]); // В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[finalDist].insert(neighbour);
            }
        }
        
        std::get<3>(pars[point]) = true; // Отмечаем нашу точку удаленной
        oldDist = std::get<0>(pars[point]) + std::get<1>(pars[point]); // Считаем приоритет, под которым наша точка лежала в словаре
        dict[oldDist].erase(point); // Удаляем точку из словаря
        if (dict[oldDist].empty()) { // Если в словаре больше нет точек с таким приоритетом
            dict.erase(oldDist); // Удаляем этот ключ из словаря
        }
    }
    
    // Восстановление пути
    std::vector<unsigned long long int> lst;
    for (unsigned long long int i = point; i != start; i = std::get<2>(pars[i])) // Прыгаем по родителям точки, пока не дойдём до старта
    {
        lst.push_back(i);
    }
    lst.push_back(start); // Добавляем старт
    std::reverse(lst.begin(), lst.end()); // Переворачиваем путь
    return lst;
}