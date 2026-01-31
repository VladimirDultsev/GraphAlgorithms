#include "bfs.h"
#include <iostream>
#include <map>
#include <utility>
#include <vector>
#include <queue>
#include <fstream>
#include "../utils/graph.h"

using namespace Graph;

std::vector<std::vector<unsigned short>> BFS(std::vector<unsigned short>& st)
{
    std::unordered_map<unsigned long long int, std::tuple<unsigned long long int, bool ,unsigned long long int>> pars;
    unsigned long long int curr;
    const unsigned long long int start = fieldToLong(st);
    std::queue<unsigned long long int> q;
    std::vector<unsigned long long int> neighbours;
    Start = clock();
    q.push(start);
    pars[start] = std::make_tuple(0, true, 0);
    while(!q.empty())
    {
        if(clock() - Start > threshold)
        {
            std::cout << "Time limit error!\n";
            std::vector<std::vector<unsigned short>> usl;
            error = true;
            return usl;
        }
        curr = q.front();
        if(curr == finish)
        {
            //cout<<"solved"<<'\n';
            break;
        }
        q.pop();
        CalcVariants(curr, neighbours);
        for(unsigned long long int l : neighbours)
        {
            if(!std::get<1>(pars[l]))
            {
                pars[l] = std::make_tuple(std::get<0>(pars[curr]) + 1, true, curr);
                q.push(l);
            }
        }
    }
    std::vector<std::vector<unsigned short>> lst;
    std::vector<unsigned short> pt(16);
    for(unsigned long long int point = std::get<2>(pars[finish]); point != 0; point = std::get<2>(pars[point])) {
        longToField(point, pt);
        lst.push_back(pt);
    }
    std::reverse(lst.begin(), lst.end());
    lst.push_back({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0});
    return lst;
}