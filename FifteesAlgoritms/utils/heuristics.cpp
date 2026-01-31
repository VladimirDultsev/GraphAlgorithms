#include "heuristics.h"
#include <cmath>
#include <mutex>
#include <vector>

#include "graph.h"
using namespace Graph;
double DoubleAStarHeuristic(unsigned long long int f, const std::vector<unsigned short>& Purpose)
{
    std::vector<unsigned short> field(16);
    longToField(f, field);
    double cnt = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (field[i] != 0)
        {
            cnt += abs((i % 4) - (Purpose[field[i]] % 4)) + abs((i / 4) - (Purpose[field[i]] / 4));// /1.07
        }
    }
    return cnt;
}

void calcPurposes(const std::vector<unsigned short>& st, std::vector<std::vector<unsigned short>>& Purpose)
{
    //0-й A* - прямой, 1-й A* - обратный
    Purpose[0] = {15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
    for(unsigned short i = 0; i < 16; ++i)
    {
        Purpose[1][st[i]] = i;
    }
}

double AStarHeuristic(unsigned long long int f)
{
    std::vector<unsigned short> Field(16);
    longToField(f, Field);
    double cnt = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (Field[i] != 0)
        {
            cnt += abs(i % 4 - (Field[i] - 1) % 4) + abs(i / 4 - (Field[i] - 1) / 4);
        }
    }
    constexpr double delta = 0.4;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 1; j < 4; j += 4)
        {
            if (Field[0 + i] != 0 && Field[j + i] != 0 && Field[0 + i] > Field[j + i]/* && (((f[0 + i] - 1) % 4) == i) && (((f[j + i] - 1) % 4) == i)*/)
            {
                cnt += delta;
            }
        }
        if (Field[4 + i] != 0 && Field[8 + i] != 0 && Field[4 + i] > Field[8 + i] /*&& (((f[4 + i] - 1) % 4) == i) && (((f[8 + i] - 1) % 4)== i)*/)
        {
            cnt += delta;
        }
        if (Field[4 + i] != 0 && Field[12 + i] != 0 && Field[4 + i] > Field[12 + i] /*&& (((f[4 + i] - 1) % 4) == i) && (((f[12 + i] - 1) % 4) == i)*/)
        {
            cnt += delta;
        }
        if (Field[8 + i] != 0 && Field[12 + i] != 0 && Field[8 + i] > Field[12 + i] /*&& (((f[8 + i] - 1) % 4) == i) && (((f[12 + i] - 1) % 4) == i)*/)
        {
            cnt += delta;
        }
    }
    for (int i = 0; i < 4; i += 4)
    {
        for (int j = 1; j < 4; j++)
        {
            if (Field[0 + i] != 0 && Field[i + j] != 0 && Field[0 + i] > Field[i + j]/* && (((f[0 + i] - 1) / 4) == i / 4) && (((f[i + j] - 1) / 4) == i / 4)*/)
            {
                cnt += delta;
            }
        }
        if (Field[1 + i] != 0 && Field[2 + i] != 0 && Field[1 + i] > Field[2 + i]/* && (((f[1 + i] - 1) / 4) == i / 4) && ((f[2 + i] - 1) / 4 == i / 4)*/)
        {
            cnt += delta;
        }
        if (Field[1 + i] != 0 && Field[3 + i] != 0 && Field[1 + i] > Field[3 + i] /*&& (((f[1 + i] - 1) / 4) == i / 4) && (((f[3 + i] - 1) / 4) == i / 4)*/)
        {
            cnt += delta;
        }
        if (Field[2 + i] != 0 && Field[3 + i] != 0 && Field[2 + i] > Field[3 + i]/* && (((f[2 + i] - 1) / 4) == i / 4) && (((f[3 + i] - 1) / 4) == i / 4)*/)
        {
            cnt += delta;
        }
    }
    //cout<<(unsigned short)(cnt/1)<<"\n";
    return cnt;
}

// std::mutex HeuristicMutex;
// double MultithreadDoubleAStarHeuristic(unsigned long long int f, int ind)
// {
//     longToField(f);
//     cnt = 0;
//     for (int i = 0; i < 16; ++i)
//     {
//         if (Field[i] != 0)
//         {
//             {
//                 std::lock_guard<std::mutex> lock(HeuristicMutex);
//                 cnt += abs((i % 4) - (Purpose[ind][Field[i]] % 4)) + abs((i / 4) - (Purpose[ind][Field[i]] / 4));// /1.07
//             }
//         }
//     }
//     return cnt;
// }