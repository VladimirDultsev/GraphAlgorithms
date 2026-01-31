#include "bidirectional_bfs.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <utility>
#include "../utils/graph.h"

using namespace Graph;
std::vector<std::vector<unsigned short>> DoubleBFS(const std::vector<unsigned short>& st)
{
    unsigned long long int curr;
    unsigned short BFSIndex = 0;
    unsigned long long int start = fieldToLong(st);
    std::queue<unsigned long long int> q[2];
    std::unordered_map<unsigned long long int, std::tuple<unsigned long long int, bool ,unsigned long long int>> pars[2];//расстояние, была ли посещена и родитель
    std::vector<unsigned long long int> neighbours;
    Start = clock();
    q[0].push(start);
    pars[0][start] = std::make_tuple(0, true, 0);
    q[1].push(finish);
    pars[1][finish] = std::make_tuple(0, true, 0);
    while(!q[0].empty() && !q[1].empty())
    {
        if(clock() - Start > threshold)
        {
            std::cout << "Time limit error!\n";
            std::vector<std::vector<unsigned short>> usl;
            error = true;
            return usl;
        }
        BFSIndex = 1 - BFSIndex;
        curr = q[BFSIndex].front();
        if(std::get<1>(pars[1-BFSIndex][curr]))
        {
            break;
        }
        if(!BFSIndex && curr == finish || BFSIndex && curr == start)
        {
            std::cout<<"error"<<'\n';
            break;
        }
        q[BFSIndex].pop();
        CalcVariants(curr, neighbours);
        for(unsigned long long int l : neighbours)
        {
            if(!std::get<1>(pars[BFSIndex][l]))
            {
                pars[BFSIndex][l] = std::make_tuple(std::get<0>(pars[BFSIndex][curr]) + 1, true, curr);
                q[BFSIndex].push(l);
            }
        }
    }
    std::vector<std::vector<unsigned short>> lst;
    std::vector<unsigned short> pt;
    for(unsigned long long int point = curr; point != 0; point = std::get<2>(pars[0][point])) {
        longToField(point, pt);
        lst.push_back(pt);
    }
    reverse(lst.begin(), lst.end());
    for(unsigned long long int point = std::get<2>(pars[1][curr]); point != 0; point = std::get<2>(pars[1][point])) {
        longToField(point, pt);
        lst.push_back(pt);
    }
    return lst;
}
