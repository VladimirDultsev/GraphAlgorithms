#include "bidirectional_astar.h"
#include "../utils/graph.h"
#include "../utils/distance.h"
#include <algorithm>

extern bool isDejkstra; // Флаг для определения, используем ли мы эвристику

using namespace Graph;
/// 2A*
std::vector<unsigned long long int> DoubleAStar()
{
    int AStarIndex = 0;
    unsigned long long int point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;
    std::map<double, std::unordered_set<unsigned long long int>> dict[2];
    std::unordered_map<unsigned long long int, std::tuple<double, double, unsigned long long int, bool>> pars[2];

    pars[0][start] = std::make_tuple(Graph::heuristic(start, 0), 0, start, false); // Добавляем информацию о стартовой вершине для прямого обхода
    dict[0][std::get<0>(pars[0][start])].insert(start); // Добавляем старт в словарь прямого A*-а
    
    pars[1][finish] = std::make_tuple(Graph::heuristic(finish, 1), 0, finish, false); // Добавляем информацию о стартовой вершине для обратного обхода
    dict[1][std::get<0>(pars[1][finish])].insert(finish); // Добавляем финиш в словарь обратного A*-а
    
    while (!dict[0].empty() || !dict[1].empty()) // Алгоритм работает пока хоть в одном из словарей есть точки
    {
        if (!dict[0].empty() && !dict[1].empty()) { // Если в обоих словарях есть точки
            AStarIndex = (dict[0].begin()->first < dict[1].begin()->first) ? 0 : 1; // Берем точку из того словаря, где приоритет меньше
            point = *(dict[AStarIndex].begin()->second.begin()); // Текущая точка - случайная из наименьших по расстоянию точек выбранного словаря
        }
        else if (!dict[0].empty()) { // Если словарь обратного A*-а пустой
            AStarIndex = 0; // Берем точку из словаря прямого A*-а
            point = *(dict[0].begin()->second.begin()); // Текущая точка - случайная из наименьших по расстоянию точек словаря
        }
        else { // Если словарь прямого A*-а пустой
            AStarIndex = 1; // Берем точку из словаря обратного A*-а
            point = *(dict[1].begin()->second.begin()); // Текущая точка - случайная из наименьших по расстоянию точек словаря
        }

        if (pars[1 - AStarIndex].count(point) && std::get<3>(pars[1 - AStarIndex][point])) { // Если точка уже удалена из другого словаря, значит A*-ы соединились
            break; // Прекращаем работу
        }
        
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
    
    // Восстановление пути
    std::vector<unsigned long long int> path;
    for(unsigned long long int pt = point; pt != start; pt = std::get<2>(pars[0][pt])){ // Прыгаем по соседям из pars прямого A*-а пока не дойдем до старта
        path.push_back(pt);
    }
    path.push_back(start); // Добавляем старт в путь
    std::reverse(path.begin(), path.end()); // Переворачиваем путь, потому что мы шли в обратном порядке
    
    for(unsigned long long int pt = std::get<2>(pars[1][point]); pt != finish; pt = std::get<2>(pars[1][pt])){ // Прыгаем по соседям из pars обратного A*-а пока не дойдем до финиша
        path.push_back(pt);
    }
    path.push_back(finish); // Добавляем финиш в путь
    return path;
}