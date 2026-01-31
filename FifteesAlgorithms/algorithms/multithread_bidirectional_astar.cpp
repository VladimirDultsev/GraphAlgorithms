#include "multithread_bidirectional_astar.h"

#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <thread>
#include "../utils/heuristics.h"
#include "../utils/graph.h"
using namespace Graph;

static std::mutex cvMtx, mtx;
static std::condition_variable cv;
static std::atomic<bool> needToFinish{false};
static std::vector<std::vector<unsigned short>> fstPart, sndPart;
static unsigned long long int buffPoint, stPos;
static std::unordered_set<unsigned long long int> used;
static std::vector<std::vector<unsigned short>> Purpose(2, std::vector<unsigned short>(16));

void ModifiedAStar(int AStarIndex) {
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unsigned long long int point = 0, start;
    double n_dist;
    std::unordered_map<unsigned long long int, std::tuple<double, double, unsigned long long int>> pars;
    std::map<double, std::unordered_set<unsigned long long int>> dict;
    std::unordered_set<unsigned long long int> localUsed;
    std::vector<unsigned long long int> neighbours;
    if (!AStarIndex) {
        start = stPos;
    } else {
        start = finish;
    }
    pars[start] = std::make_tuple(DoubleAStarHeuristic(start, Purpose[AStarIndex]), 0, start);
    dict[std::get<0>(pars[start]) + std::get<1>(pars[start])].insert(start);
    while (!dict.empty() && !needToFinish && !error) {
        if (clock() - Start > threshold) {
            std::lock_guard<std::mutex> lock(mtx);
            needToFinish = true;
            error = true;
            cv.notify_all();
            std::cout << "Time limit error!\n";
            break;
        }
        point = *((dict.begin())->second).begin();
        if (point == 0)
        {
            std::cout << std::endl;
        }
        std::unique_lock<std::mutex> lock(mtx, std::defer_lock);
        lock.lock();
        if ((!AStarIndex && point == finish) ||(AStarIndex && point == stPos) || (used.count(point) && !localUsed.count(point))) {
            buffPoint = point;
            needToFinish = true;
            cv.notify_all();
            break;
        }
        localUsed.insert(point);
        used.insert(point);
        lock.unlock();
        CalcVariants(point, neighbours);
        for (unsigned long long int neighbour : neighbours) {
            newNeighbourDist = std::get<1>(pars[point]) + 1;
            if (!pars.count(neighbour)) {
                heuristic = DoubleAStarHeuristic(neighbour, Purpose[AStarIndex]);
                pars[neighbour] = std::make_tuple(heuristic, newNeighbourDist, point);
                finalDist = newNeighbourDist + heuristic;
                dict[finalDist].insert(neighbour);
            } else if (newNeighbourDist < std::get<1>(pars[neighbour])) {
                oldDist = std::get<0>(pars[neighbour]) + std::get<1>(pars[neighbour]);
                if (dict.count(oldDist)) {
                    dict[oldDist].erase(neighbour);
                    if (dict[oldDist].empty()) {
                        dict.erase(oldDist);
                    }
                }
                std::get<1>(pars[neighbour]) = newNeighbourDist;
                std::get<2>(pars[neighbour]) = point;
                finalDist = newNeighbourDist + std::get<0>(pars[neighbour]);
                dict[finalDist].insert(neighbour);
            }
        }
        pars[point] = std::make_tuple(std::get<0>(pars[point]), std::get<1>(pars[point]), std::get<2>(pars[point]));
        n_dist = std::get<1>(pars[point]) + std::get<0>(pars[point]);
        if (dict.count(n_dist)) {
            dict[n_dist].erase(point);
            if (dict[n_dist].empty()) {
                dict.erase(n_dist);
            }
        }
    }
    if (error) {
        return;
    }
    std::vector<unsigned short> field;
    if (!AStarIndex) {
        std::vector<unsigned long long int> tempPath;
        for (unsigned long long int pt = std::get<2>(pars[buffPoint]); pt != stPos; pt = std::get<2>(pars[pt])) {
            tempPath.push_back(pt);
        }
        tempPath.push_back(stPos);
        std::reverse(tempPath.begin(), tempPath.end());
        for (const unsigned long long int pt : tempPath) {
            longToField(pt, field);
            fstPart.push_back(field);
        }
    } else {
        std::vector<unsigned long long int> tempPath;
        for (unsigned long long int pt = std::get<2>(pars[buffPoint]); pt != finish; pt = std::get<2>(pars[pt])) {
            tempPath.push_back(pt);
        }
        tempPath.push_back(finish);
        for (const unsigned long long int pt : tempPath) {
            longToField(pt, field);
            sndPart.push_back(field);
        }
    }
}

std::vector<std::vector<unsigned short>> Multithread_2AStar(std::vector<unsigned short>& st) {
    fstPart.clear();
    sndPart.clear();
    used.clear();
    needToFinish = false;
    error = false;
    stPos = fieldToLong(st);
    Start = clock();
    calcPurposes(st, Purpose);
    std::thread ModifiedAstar1(ModifiedAStar, 0);
    std::thread ModifiedAstar2(ModifiedAStar, 1);
    {
        std::unique_lock<std::mutex> lk(cvMtx);
        cv.wait(lk, []{ return needToFinish.load(); });
    }
    if (ModifiedAstar1.joinable()) {
        ModifiedAstar1.join();
    }
    if (ModifiedAstar2.joinable()) {
        ModifiedAstar2.join();
    }
    if (error) {
        return {};
    }
    std::vector<std::vector<unsigned short>> path;
    path.insert(path.end(), fstPart.begin(), fstPart.end());
    // std::vector<unsigned short> field;
    // longToField(buffPoint, field);
    // path.push_back(field);
    path.insert(path.end(), sndPart.begin(), sndPart.end());
    // for (auto& f: path)
    // {
    //     print(fieldToLong(f));
    // }
    return path;
}