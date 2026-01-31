#include "bidirectional_astar.h"
#include <iostream>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>
#include "../utils/heuristics.h"
#include "../utils/graph.h"

using namespace Graph;

std::vector<std::vector<unsigned short>> DoubleAStar(std::vector<unsigned short>& st)
{
    std::vector<std::vector<unsigned short>> Purpose(2, std::vector<unsigned short>(16));
    std::unordered_map<unsigned long long int, std::
    tuple<double, unsigned long long int, unsigned long long int, bool>>pars[2];//эвристика, minDist (чистое расстояние без эвристики), родитель
    std::map<unsigned long long int, std::unordered_set<unsigned long long int>> dict[2];
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unsigned long long int n_dist, point = 0, start = fieldToLong(st);
    std::unordered_set<unsigned long long int> EmptySet;
    std::vector<unsigned long long int> neighbours;
    calcPurposes(st, Purpose);
    unsigned short AStarIndex = 0;
    dict[0].clear();
    dict[1].clear();
    Start = clock();
    pars[0][start] = std::make_tuple(DoubleAStarHeuristic(start, Purpose[AStarIndex]), 0, start, false);
    dict[0][std::get<0>(pars[0][start])].insert(start);
    AStarIndex = 1;
    pars[1][finish] = std::make_tuple(DoubleAStarHeuristic(finish, Purpose[AStarIndex]), 0, finish, false);
    dict[1][std::get<0>(pars[1][finish])].insert(finish);
    while (!dict[0].empty() && !dict[1].empty())
    {
        if(clock() - Start > threshold)
        {
            std::cout << "Time limit error!\n";
            std::vector<std::vector<unsigned short>> usl;
            error = true;
            return usl;
        }
        if(!dict[0].empty() && !dict[1].empty())
        {
            point = ((dict[0].begin())->first < (dict[1].begin())->first? *((dict[0].begin())->second).begin(): *((dict[1].begin())->second).begin());
            AStarIndex = ((dict[0].begin())->first < (dict[1].begin())->first? 0: 1);
        }
        else if(dict[0].empty())
        {
            point = *((dict[1].begin())->second).begin();
            AStarIndex = 1;
        }
        else
        {
            point = *((dict[0].begin())->second).begin();
            AStarIndex = 0;
        }
        if (std::get<3>(pars[1 - AStarIndex][point]))
        {
            break;
        }
        if(AStarIndex == 0 && point == finish || AStarIndex == 1 && point == start)
        {
            break;
        }
        CalcVariants(point, neighbours);
        for (unsigned long long int neighbour : neighbours)
        {
            newNeighbourDist = std::get<1>(pars[AStarIndex][point]) + 1;// Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
            if (!pars[AStarIndex].count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = DoubleAStarHeuristic(neighbour, Purpose[AStarIndex]);// Считаем эвристику до него
                pars[AStarIndex][neighbour] = std::make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic;// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
            else if (newNeighbourDist < std::get<1>(pars[AStarIndex][neighbour]))// Если найден более короткий путь до соседа
            {
                oldDist = std::get<0>(pars[AStarIndex][neighbour]) + std::get<1>(pars[AStarIndex][neighbour]);// Считаем приоритет, под которым сосед был положен в словарь раньше
                dict[AStarIndex][oldDist].erase(neighbour);// Удаляем соседа из словаря
                if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
                    dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
                }
                std::get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist;// Обновляем расстояние до соседа
                std::get<2>(pars[AStarIndex][neighbour]) = point;// Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + std::get<0>(pars[AStarIndex][neighbour]);// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }
        pars[AStarIndex][point] = std::make_tuple(std::get<0>(pars[AStarIndex][point]), std::get<1>(pars[AStarIndex][point]), std::get<2>(pars[AStarIndex][point]), true);
        n_dist = std::get<1>(pars[AStarIndex][point]) + std::get<0>(pars[AStarIndex][point]);
        if (dict[AStarIndex].find(n_dist) != dict[AStarIndex].end())
        {
            dict[AStarIndex][n_dist].erase(point);
        }
        if (dict[AStarIndex][n_dist].empty())
        {
            dict[AStarIndex].erase(n_dist);
        }
    }
    std::vector<std::vector<unsigned short>> lst;
    std::vector<unsigned short> pt;
    for (unsigned long long int i = point; i != start; i = std::get<2>(pars[0][i]))
    {
        longToField(i, pt);
        lst.push_back(pt);
    }
    reverse(lst.begin(), lst.end());
    for(unsigned long long int i = std::get<2>(pars[1][point]); i != finish; i = std::get<2>(pars[1][i]))
    {
        longToField(i, pt);
        lst.push_back(pt);
    }
    longToField(finish, pt);
    lst.push_back(pt);
    return lst;
}