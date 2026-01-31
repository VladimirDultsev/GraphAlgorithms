#ifndef GRAPH_H
#define GRAPH_H
#include <vector>

namespace Graph {
    extern unsigned long long int start;
    extern unsigned long long int finish;
    extern unsigned long long int Start;
    extern unsigned long long int threshold;
    extern bool error;

    void CalcVariants(unsigned long long int f, std::vector<unsigned long long int>& ls);

    unsigned long long int fieldToLong(const std::vector<unsigned short>& f);

    void longToField(unsigned long long int num, std::vector<unsigned short>& f);

    double heuristic(unsigned long long int point, int AStarIndex);

    void setStartFinish(unsigned long long int s, unsigned long long int f);

    void print(unsigned long long int l);

    bool amountOfInversions(std::vector<unsigned short>& f);
}


#endif
