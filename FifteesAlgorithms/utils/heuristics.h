#ifndef HEURISTICS_H
#define HEURISTICS_H

#include <vector>

double DoubleAStarHeuristic(unsigned long long int f, const std::vector<unsigned short>& Purpose);

void calcPurposes(const std::vector<unsigned short>& st, std::vector<std::vector<unsigned short>>& Purpose);

double AStarHeuristic(unsigned long long int f);

#endif
