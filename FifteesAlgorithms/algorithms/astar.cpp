#include "astar.h"
#include <iostream>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>
#include "../utils/heuristics.h"
#include "../utils/graph.h"

using namespace Graph;

std::vector<std::vector<unsigned short>> AStar(const std::vector<unsigned short>& st)
{
    std::unordered_map<unsigned long long int, std::tuple<double, unsigned long long int, unsigned long long int>>pars;//эвристика, minDist (чистое расстояние без эвристики), родитель
    std::map<unsigned long long int, std::unordered_set<unsigned long long int>> dict;// Очередь с приоритетом (Упорядоченный словарь неупорядоченных множеств)
    double finalDist, oldDist, heuristic, newNeighbourDist, currKey;
    unsigned long long int point = 0, start = fieldToLong(st);
    std::unordered_set<unsigned long long int> EmptySet;
    std::vector<unsigned long long int> neighbours;
    Start = clock();// Начинаем замер времени
    pars[start] = std::make_tuple(AStarHeuristic(start), 0, start);// Добавляем информацию о стартовой вершине
    dict[std::get<0>(pars[start])].insert(start);// Добавляем стартовую вершину в очередь
    while (!dict.empty())// Пока в очереди есть точки
    {
        if(clock() - Start > threshold)// Если алгоритм выполняется слишком долго
        {
            std::cout << "Time limit error!\n";
            std::vector<std::vector<unsigned short>> usl;
            error = true;// Ставим глобальную переменную ошибки в true
            return usl;// Возвращаем пустой путь
        }
        point = *((dict.begin())->second).begin();// Берем случайную из точек с наибольшим приоритетом в очереди
        currKey = dict.begin()->first;
        if (point == finish)// Если это финиш - завершаемся
        {
            break;
        }
        CalcVariants(point, neighbours);// Считаем соседние состояния
        for (unsigned long long int neighbour : neighbours)// Перебираем соседей
        {
            newNeighbourDist = std::get<1>(pars[point]) + 1;// Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа, в данном случае расстояние до соседа 1)
            if (!pars.count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = AStarHeuristic(neighbour);// Считаем эвристику до него
                pars[neighbour] = std::make_tuple(heuristic, newNeighbourDist, point);// Обновляем информацию о вершине
                finalDist = newNeighbourDist + heuristic;// Прибавляем эвристику к расстоянию до соседа
                dict[finalDist].insert(neighbour);// Добавляем вершину в очередь
            }
            else if (newNeighbourDist < std::get<1>(pars[neighbour]))// Если найден более короткий путь до соседа
            {
                oldDist = std::get<0>(pars[neighbour]) + std::get<1>(pars[neighbour]);// Считаем приоритет, под которым сосед был положен в очередь раньше
                dict[oldDist].erase(neighbour);// Удаляем соседа из очереди
                if (dict[oldDist].empty()) {// Если в очереди больше нет точек с таким приоритетом
                    dict.erase(oldDist);// Удаляем этот приоритет из очереди
                }
                std::get<1>(pars[neighbour]) = newNeighbourDist;// Обновляем расстояние до соседа
                std::get<2>(pars[neighbour]) = point;// Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + std::get<0>(pars[neighbour]);// В очередь кладем соседа под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[finalDist].insert(neighbour);
            }
        }
        if (dict.find(currKey) != dict.end())
        {
            dict[currKey].erase(point);// Удаляем рассмотренную точку из словаря
        }
        if (dict[currKey].empty())// Если в очереди больше нет точек с таким приоритетом
        {
            dict.erase(currKey);// Удаляем этот приоритет из очереди
        }
    }
    // Восстанавливаем путь
    std::vector<std::vector<unsigned short>> lst;
    std::vector<unsigned short> pt;
    // Прыгаем по родителям пока не дойдём до старта
    for (unsigned long long int i = point; i != start; i = std::get<2>(pars[i]))
    {
        longToField(i, pt);
        lst.push_back(pt);
    }
    // Добавляем старт в путь
    longToField(start, pt);
    lst.push_back(pt);
    // Переворачиваем путь,ттак как мы шли в обратном порядке
    reverse(lst.begin(), lst.end());
    return lst;
}