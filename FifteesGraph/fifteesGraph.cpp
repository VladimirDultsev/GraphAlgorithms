#include "fifteesGraph.h"
#include <cmath>
#include <algorithm>

FifteesGraph::FifteesGraph(uint64_t goal) : goal_(goal) {}

/// Раскодирует состояние поля num в массив f
void FifteesGraph::longToField(uint64_t num, std::vector<unsigned short>& f) {
    if (f.size() != 16) f.resize(16);
    for (int i = 0; i < 16; ++i)
    {
        f[15 - i] = static_cast<unsigned short>((num >> i * 4) % 16);
    }
}

/// Возвращает закодированное в число состояние поля f
uint64_t FifteesGraph::fieldToLong(const std::vector<unsigned short>& f) {
    uint64_t Res = 0;
    for (int i = 0; i < 16; ++i)
    {
        Res |= static_cast<unsigned long long int>(f[i]) << (60 - i * 4);
    }
    return Res;
}

/// Возвращает поле st с поменянными местами ячейками на позициях i и zeroPos
uint64_t FifteesGraph::swap(uint64_t st, unsigned short i, unsigned short zeroPos) {
    uint64_t num = (static_cast<uint64_t>(15)<<4*(15 - i)&st)>>4*(15 - i);
    st -= (static_cast<uint64_t>(15)<<4*(15 - i))&st;
    st += num<<4*(15 - zeroPos);
    return st;
}

/// Вспомогательная функция маппинга для эвристики 2A*
void FifteesGraph::calcPurposes(const std::vector<unsigned short> &st, std::vector<std::vector<unsigned short>>& Purpose) {
    //0-й A* - прямой, 1-й A* - обратный
    Purpose[0] = {15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
    for(unsigned short i = 0; i < 16; ++i)
    {
        Purpose[1][st[i]] = i;
    }
}

/// Возвращает массив соседей для вершины state
std::vector<uint64_t> FifteesGraph::getNeighbors(const uint64_t& state) const {
    std::vector<uint64_t> neighbors;
    unsigned short zeroPos = 0;
    // Ищем нулевую ячейку
    for (unsigned short i = 0; i < 16; ++i)
    {
        if((static_cast<uint64_t>(15)<<4*i&state)>>4*i == 0)// если ноль - это ноль
        {
            zeroPos = 15 - i;
            break;
        }
    }

    // Проверяем возможность передвижения пустой ячейки вверх
    if (zeroPos >= 4)
    {
        // Если возможно - добавляем соседа
        neighbors.push_back(swap(state, zeroPos - 4, zeroPos));
    }

    // Проверяем возможность передвижения пустой ячейки вниз
    if (zeroPos <= 11)
    {
        // Если возможно - добавляем соседа
        neighbors.push_back(swap(state, zeroPos + 4, zeroPos));
    }

    // Проверяем возможность передвижения пустой ячейки влево
    if (!(zeroPos % 4))
    {
        // Если возможно - добавляем соседа
        neighbors.push_back(swap(state, zeroPos - 1, zeroPos));
    }

    // Проверяем возможность передвижения пустой ячейки вправо
    if (!((zeroPos + 1) % 4))
    {
        // Если возможно - добавляем соседа
        neighbors.push_back(swap(state, zeroPos + 1, zeroPos));
    }

    return neighbors;
}

/// Эвристика для 2A*
double FifteesGraph::DoubleAStarHeuristic(uint64_t f, const std::vector<unsigned short> &Purpose) {
    std::vector<unsigned short> field(16);
    FifteesGraph::longToField(f, field);
    double cnt = 0;

    // Считаем инверсии
    for (int i = 0; i < 16; ++i)
    {
        if (field[i] != 0)
        {
            cnt += abs((i % 4) - (Purpose[field[i]] % 4)) + abs((i / 4) - (Purpose[field[i]] / 4));// /1.07
        }
    }
    return cnt;
}

/// Эвристика для A*
double FifteesGraph::heuristic(const uint64_t& state, const uint64_t& goal) const {
    std::vector<unsigned short> Field(16);
    FifteesGraph::longToField(state, Field);
    double cnt = 0;

    // Считаем инверсии
    for (int i = 0; i < 16; ++i)
    {
        if (Field[i] != 0)
        {
            cnt += abs(i % 4 - (Field[i] - 1) % 4) + abs(i / 4 - (Field[i] - 1) / 4);
        }
    }

//    constexpr double delta = 0.4;
//    for (int i = 0; i < 4; i++)
//    {
//        for (int j = 1; j < 4; j += 4)
//        {
//            if (Field[0 + i] != 0 && Field[j + i] != 0 && Field[0 + i] > Field[j + i]/* && (((f[0 + i] - 1) % 4) == i) && (((f[j + i] - 1) % 4) == i)*/)
//            {
//                cnt += delta;
//            }
//        }
//        if (Field[4 + i] != 0 && Field[8 + i] != 0 && Field[4 + i] > Field[8 + i] /*&& (((f[4 + i] - 1) % 4) == i) && (((f[8 + i] - 1) % 4)== i)*/)
//        {
//            cnt += delta;
//        }
//        if (Field[4 + i] != 0 && Field[12 + i] != 0 && Field[4 + i] > Field[12 + i] /*&& (((f[4 + i] - 1) % 4) == i) && (((f[12 + i] - 1) % 4) == i)*/)
//        {
//            cnt += delta;
//        }
//        if (Field[8 + i] != 0 && Field[12 + i] != 0 && Field[8 + i] > Field[12 + i] /*&& (((f[8 + i] - 1) % 4) == i) && (((f[12 + i] - 1) % 4) == i)*/)
//        {
//            cnt += delta;
//        }
//    }
//    for (int i = 0; i < 4; i += 4)
//    {
//        for (int j = 1; j < 4; j++)
//        {
//            if (Field[0 + i] != 0 && Field[i + j] != 0 && Field[0 + i] > Field[i + j]/* && (((f[0 + i] - 1) / 4) == i / 4) && (((f[i + j] - 1) / 4) == i / 4)*/)
//            {
//                cnt += delta;
//            }
//        }
//        if (Field[1 + i] != 0 && Field[2 + i] != 0 && Field[1 + i] > Field[2 + i]/* && (((f[1 + i] - 1) / 4) == i / 4) && ((f[2 + i] - 1) / 4 == i / 4)*/)
//        {
//            cnt += delta;
//        }
//        if (Field[1 + i] != 0 && Field[3 + i] != 0 && Field[1 + i] > Field[3 + i] /*&& (((f[1 + i] - 1) / 4) == i / 4) && (((f[3 + i] - 1) / 4) == i / 4)*/)
//        {
//            cnt += delta;
//        }
//        if (Field[2 + i] != 0 && Field[3 + i] != 0 && Field[2 + i] > Field[3 + i]/* && (((f[2 + i] - 1) / 4) == i / 4) && (((f[3 + i] - 1) / 4) == i / 4)*/)
//        {
//            cnt += delta;
//        }
//    }
    //cout<<(unsigned short)(cnt/1)<<"\n";
    return cnt;
}

/// Проверяет является ли точка финишем
bool FifteesGraph::isGoal(const uint64_t& state, const uint64_t& goal) const {
    return state == goal;
}


