#include "multithread_bidirectional_bfs.h"
#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <thread>
#include "../utils/graph.h"
using namespace Graph;

static std::mutex cvMtx, mtx;
static std::condition_variable cv;
static std::atomic<bool> needToFinish{false};
static std::vector<std::vector<unsigned short>> fstPart, sndPart;
static unsigned long long int buffPoint, stPos;
static std::unordered_set<unsigned long long int> used;

void ModifiedBFS(int BFSIndex)
{
    std::unordered_map<unsigned long long int, std::tuple<unsigned long long int, bool ,unsigned long long int>> pars;
    unsigned long long int curr, start;
    std::vector<unsigned long long int> neighbours;
    if (!BFSIndex)
    {
        start = stPos;
    }
    else
    {
        start = finish;
    }
    std::queue<unsigned long long int> q;
    q.push(start);
    pars[start] = std::make_tuple(0, true, 0);
    while(!q.empty() && !needToFinish && !error)
    {
        if(clock() - Start > threshold)
        {
            mtx.lock();
            needToFinish = true;
            error = true;
            mtx.unlock();
            cv.notify_all();
            std::cout << "Time limit error!\n";
            break;
        }
        curr = q.front();
        std::unique_lock<std::mutex> lock(mtx, std::defer_lock);
        lock.lock();
        if((!BFSIndex && curr == finish) || (BFSIndex && curr == stPos) || used.count(curr))
        {
            if ((!BFSIndex && curr == finish) || (BFSIndex && curr == start))
            {
                std::cout << "Solved by one bfs\n";
            }
            buffPoint = curr;
            needToFinish = true;
            mtx.unlock();
            cv.notify_all();
            break;
        }
        used.insert(curr);
        lock.unlock();
        q.pop();
        CalcVariants(curr, neighbours);
        for(unsigned long long int l : neighbours)
        {
            //print(l);
            if(!std::get<1>(pars[l]))
            {
                pars[l] = std::make_tuple(std::get<0>(pars[curr]) + 1, true ,curr);
                q.push(l);
            }
        }
    }
    if (error)
    {
        return;
    }
    std::vector<unsigned short> field;
    for(unsigned long long int pt = buffPoint; pt != start; pt = std::get<2>(pars[pt]))
    {
        longToField(pt, field);
        if (!BFSIndex)
        {
            fstPart.push_back(field);
        }
        else
        {
            sndPart.push_back(field);
        }
    }
}

std::vector<std::vector<unsigned short>> Multithread_2BFS(std::vector<unsigned short>& st){
    fstPart.clear();
    sndPart.clear();
    used.clear();
    needToFinish = false;
    error = false;
    stPos = fieldToLong(st);
    Start = clock();

    std::thread ModifiedBFS1(ModifiedBFS, 0);
    std::thread ModifiedBFS2(ModifiedBFS, 1);
    {
        std::unique_lock<std::mutex> lk(cvMtx);
        cv.wait(lk, []{ return needToFinish.load(); });
    }

    if(ModifiedBFS1.joinable()){
        ModifiedBFS1.join();
    }
    if(ModifiedBFS2.joinable()){
        ModifiedBFS2.join();
    }

    if(error){
        return {};
    }

    std::vector<std::vector<unsigned short>> path;
    path.insert(path.end(), fstPart.begin(), fstPart.end());
    path.push_back(st);
    reverse(path.begin(), path.end());
    path.insert(path.end(), sndPart.begin(), sndPart.end());
    path.push_back({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0});

    return path;
}