#include <iostream>
#include <unordered_set>
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <stack>
#include <fstream>
#include <utility>
#include <thread>
#include <mutex>
#include <functional>
#include <random>
#include "json.hpp"
#include "rapidjson-master/include/rapidjson/document.h"
#include "rapidjson-master/include/rapidjson/writer.h"
#include "rapidjson-master/include/rapidjson/stringbuffer.h"
#include "rapidjson-master/include/rapidjson/istreamwrapper.h"

/// Структура хеширования пар для хранения пар плоскостей
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2;
    }
};

using namespace std;
using json = nlohmann::json;
map<double, unordered_set<unsigned long long int>> dict[2];
unordered_map<unsigned long long int, tuple<double, double, unsigned long long int, bool>>pars[2];//эвристика, minDist (чистое расстояние без эвристики), родитель
unordered_map<unsigned long long int, unsigned long long int> pars1;//родитель
//unsigned short AStarIndex = 0;
unordered_map<unsigned long long int, pair<double, double>> points;
unordered_map<pair<double, double>, int, pair_hash> Indices;
unordered_map<unsigned long long int, unordered_set<unsigned long long int>> adjList;
unsigned long long int start, finish;
unordered_map<pair<double, double>, unordered_set<pair<double, double>, pair_hash>, pair_hash> adjListCoords;
unordered_map<pair<double, double>, unordered_set<pair<double, double>, pair_hash>, pair_hash> adjListCoords1;
vector<pair<double, double>> pts;
mutex mtx;
unordered_set<unsigned long long int> used;
atomic<bool> needToFinish{false};
mutex cv_mtx;
condition_variable cv;
unsigned long long int buffPoint;
bool isDejkstra;

#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI
double calcGPSDistance(double longitude_new, double latitude_new, double longitude_old, double latitude_old)
{
    double  lat_new = latitude_old * GRADOS_RADIANES;
    double  lat_old = latitude_new * GRADOS_RADIANES;
    double  lat_diff = (latitude_new-latitude_old) * GRADOS_RADIANES;
    double  lng_diff = (longitude_new-longitude_old) * GRADOS_RADIANES;

    double  a = sin(lat_diff/2) * sin(lat_diff/2) +
                cos(lat_new) * cos(lat_old) *
                sin(lng_diff/2) * sin(lng_diff/2);
    double  c = 2 * atan2(sqrt(a), sqrt(1-a));

    double  distance = RADIO_TERRESTRE * c;

    // std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "

    return abs(distance);
}
double Heuristic(unsigned long long int point, int AStarIndex)
{
    if(isDejkstra){
        return 0;
    }
    if(AStarIndex){// Если нас вызывает обратный A*
        return calcGPSDistance(points[start].first, points[start].second, points[point].first, points[point].second);
    }else{
        return calcGPSDistance(points[finish].first, points[finish].second, points[point].first, points[point].second);
    }
}
void ModifiedAStar(int AStarIndex)
{
    unsigned long long int point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unordered_set<unsigned long long int> EmptySet;
    unsigned short useless;
    if(!AStarIndex){
        pars[AStarIndex][start] = make_tuple(Heuristic(start, AStarIndex), 0, start, false);
        dict[AStarIndex][get<0>(pars[AStarIndex][start])].insert(start);
    }else{
        pars[AStarIndex][finish] = make_tuple(Heuristic(finish, AStarIndex), 0, finish, false);
        dict[AStarIndex][get<0>(pars[AStarIndex][finish])].insert(finish);
    }
    while (!dict[AStarIndex].empty() && !needToFinish)
    {
        point = *((dict[AStarIndex].begin())->second).begin();
        mtx.lock();
        if ((!AStarIndex && point == finish) || (AStarIndex && point == start) || used.count(point)) {
            buffPoint = point;
            needToFinish = true;
            mtx.unlock();
            cv.notify_all();
            return;
        }
        used.insert(point);
        mtx.unlock();
        for (unsigned long long int neighbour : adjList[point])// Перебираем соседей точки
        {
            newNeighbourDist = get<1>(pars[AStarIndex][point]) + // Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
                               calcGPSDistance(points[point].first, points[point].second, points[neighbour].first, points[neighbour].second);
            if (!pars[AStarIndex].count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = Heuristic(neighbour, AStarIndex);// Считаем эвристику до него
                pars[AStarIndex][neighbour] = make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic;// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
            else if (newNeighbourDist < get<1>(pars[AStarIndex][neighbour]))// Если найден более короткий путь до соседа
            {
                oldDist = get<0>(pars[AStarIndex][neighbour]) + get<1>(pars[AStarIndex][neighbour]);// Считаем приоритет, под которым сосед был положен в словарь раньше
                dict[AStarIndex][oldDist].erase(neighbour);// Удаляем соседа из словаря
                if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
                    dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
                }
                get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist;// Обновляем расстояние до соседа
                get<2>(pars[AStarIndex][neighbour]) = point;// Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + get<0>(pars[AStarIndex][neighbour]);// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }
        get<3>(pars[AStarIndex][point]) = true;// Отмечаем нашу точку удаленной
        oldDist = get<0>(pars[AStarIndex][point]) + get<1>(pars[AStarIndex][point]);// Считаем приоритет, под которым наша точка лежала в словаре
        dict[AStarIndex][oldDist].erase(point);// Удаляем точку из словаря
        if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
            dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
        }
    }
//    vector<unsigned long long int> lst;
//    for (unsigned long long int i = point; i != start; i = get<2>(pars[0][i]))
//    {
//        lst.push_back(i);
//    }
//    reverse(lst.begin(), lst.end());
    //lst.push_back(Field);
    /*
    for(int i = 0; i < lst.size(); ++i)
    {
        printField(lst[i]);
    }*/
    //return lst;
}
vector<unsigned long long int> AStar()
{
    int AStarIndex = 0;
    unsigned long long int point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unordered_set<unsigned long long int> EmptySet;
    unsigned short useless;
    pars[AStarIndex].clear();
    dict[AStarIndex].clear();
    pars[AStarIndex][start] = make_tuple(Heuristic(start, AStarIndex), 0, start, false);
    dict[AStarIndex][get<0>(pars[0][start])].insert(start);
    while (!dict[AStarIndex].empty())
    {
        point = *((dict[AStarIndex].begin())->second).begin();
        if (point == finish)
        {
            //cout << "solved\n";
            break;
        }
        for (unsigned long long int neighbour : adjList[point])// Перебираем соседей точки
        {
            newNeighbourDist = get<1>(pars[AStarIndex][point]) + // Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
                               calcGPSDistance(points[point].first, points[point].second, points[neighbour].first, points[neighbour].second);
            if (!pars[AStarIndex].count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = Heuristic(neighbour, AStarIndex);// Считаем эвристику до него
                pars[AStarIndex][neighbour] = make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic;// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
            else if (newNeighbourDist < get<1>(pars[AStarIndex][neighbour]))// Если найден более короткий путь до соседа
            {
                oldDist = get<0>(pars[AStarIndex][neighbour]) + get<1>(pars[AStarIndex][neighbour]);// Считаем приоритет, под которым сосед был положен в словарь раньше
                dict[AStarIndex][oldDist].erase(neighbour);// Удаляем соседа из словаря
                if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
                    dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
                }
                get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist;// Обновляем расстояние до соседа
                get<2>(pars[AStarIndex][neighbour]) = point;// Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + get<0>(pars[AStarIndex][neighbour]);// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }
        get<3>(pars[AStarIndex][point]) = true;// Отмечаем нашу точку удаленной
        oldDist = get<0>(pars[AStarIndex][point]) + get<1>(pars[AStarIndex][point]);// Считаем приоритет, под которым наша точка лежала в словаре
        dict[AStarIndex][oldDist].erase(point);// Удаляем точку из словаря
        if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
            dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
        }
    }
    vector<unsigned long long int> lst;
    for (unsigned long long int i = point; i != start; i = get<2>(pars[AStarIndex][i]))
    {
        lst.push_back(i);
    }
    lst.push_back(start);
    reverse(lst.begin(), lst.end());
    return lst;
}
vector<unsigned long long int> DoubleAStar()
{
    int AStarIndex = 0;
    unsigned long long int point = 0;
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unordered_set<unsigned long long int> EmptySet;
    pars[0].clear();
    pars[1].clear();
    dict[0].clear();
    dict[1].clear();
    pars[0][start] = make_tuple(Heuristic(start, 0), 0, start, false);
    dict[0][get<0>(pars[0][start])].insert(start);// Добавляем старт в словарь прямого A*-а под приоритетом в виде только эвристики, потому что расстояние от старта до старта 0
    pars[1][finish] = make_tuple(Heuristic(finish, 1), 0, finish, false);
    dict[1][get<0>(pars[1][finish]) + get<1>(pars[1][finish])].insert(finish);// Добавляем финиш в словарь обратного A*-а под приоритетом в виде только эвристики, потому что расстояние от финиша до финиша 0
    while (!dict[0].empty() || !dict[1].empty())// Алгоритм работает пока хоть в одном из словарей есть точки
    {
        if (!dict[0].empty() && !dict[1].empty()) {// Если в обоих словарях есть точки
            AStarIndex = (dict[0].begin()->first < dict[1].begin()->first) ? 0 : 1;// Берем точку из того словаря, где приоритет меньше
            point = *(dict[AStarIndex].begin()->second.begin());
        }
        else if (!dict[0].empty()) {// Если словарь обратного A*-а пустой
            AStarIndex = 0;// Берем точку из словаря прямого A*-а
            point = *(dict[0].begin()->second.begin());
        }
        else {// Если словарь прямого A*-а пустой
            AStarIndex = 1;// Берем точку из словаря обратного A*-а
            point = *(dict[1].begin()->second.begin());
        }
        if (pars[1 - AStarIndex].count(point) && get<3>(pars[1 - AStarIndex][point])) {// Если точка уже удалена из другого словаря, значит A*-ы соединились
            break;// Прекращаем работу
        }
        for (unsigned long long int neighbour : adjList[point])// Перебираем соседей точки
        {
            newNeighbourDist = get<1>(pars[AStarIndex][point]) + // Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
                           calcGPSDistance(points[point].first, points[point].second, points[neighbour].first, points[neighbour].second);
            if (!pars[AStarIndex].count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = Heuristic(neighbour, AStarIndex);// Считаем эвристику до него
                pars[AStarIndex][neighbour] = make_tuple(heuristic, newNeighbourDist, point, false);
                finalDist = newNeighbourDist + heuristic;// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
            else if (newNeighbourDist < get<1>(pars[AStarIndex][neighbour]))// Если найден более короткий путь до соседа
            {
                oldDist = get<0>(pars[AStarIndex][neighbour]) + get<1>(pars[AStarIndex][neighbour]);// Считаем приоритет, под которым сосед был положен в словарь раньше
                dict[AStarIndex][oldDist].erase(neighbour);// Удаляем соседа из словаря
                if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
                    dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
                }
                get<1>(pars[AStarIndex][neighbour]) = newNeighbourDist;// Обновляем расстояние до соседа
                get<2>(pars[AStarIndex][neighbour]) = point;// Теперь мы - родитель этого соседа
                finalDist = newNeighbourDist + get<0>(pars[AStarIndex][neighbour]);// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
                dict[AStarIndex][finalDist].insert(neighbour);
            }
        }
        get<3>(pars[AStarIndex][point]) = true;// Отмечаем нашу точку удаленной
        oldDist = get<0>(pars[AStarIndex][point]) + get<1>(pars[AStarIndex][point]);// Считаем приоритет, под которым наша точка лежала в словаре
        dict[AStarIndex][oldDist].erase(point);// Удаляем точку из словаря
        if (dict[AStarIndex][oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
            dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
        }
    }
    vector<unsigned long long int> path;
    for(unsigned long long int pt = point; pt != start; pt = get<2>(pars[0][pt])){// Прыгаем по соседям из pars прямого A*-а пока не дойдем до старта
        path.push_back(pt);
    }
    path.push_back(start);// Добавляем старт в путь
    reverse(path.begin(), path.end());// Переворачиваем путь, потому что мы шли в обратном порядке
    for(unsigned long long int pt = get<2>(pars[1][point]); pt != finish; pt = get<2>(pars[1][pt])){// Прыгаем по соседям из pars обратного A*-а пока не дойдем до старта
        path.push_back(pt);
    }
    path.push_back(finish);// Добавляем финиш в путь
    return path;
}
vector<unsigned long long int> Dejkstra(){
    vector<double> m_dist(adjList.size(), numeric_limits<double>::max());
    map<double, unordered_set<unsigned long long int>> dict1;
    dict1[0].insert(start);
    m_dist[start] = 0;
    unsigned long long int curr;
    double currDist;
    while(!dict1.empty()){
        currDist = dict1.begin()->first;
        curr = *dict1.begin()->second.begin();
        if(curr == finish){
            //cout << "solved\n";
            break;
        }
        for(unsigned long long int neighbour: adjList[curr]){
            if(m_dist[neighbour] == numeric_limits<double>::max()){
                m_dist[neighbour] = m_dist[curr] + calcGPSDistance(points[curr].first, points[curr].second, points[neighbour].first, points[neighbour].second);
                dict1[m_dist[neighbour]].insert(neighbour);
                pars1[neighbour] = curr;
            }
            else if(m_dist[curr] + calcGPSDistance(points[curr].first, points[curr].second, points[neighbour].first, points[neighbour].second) < m_dist[neighbour]){
                dict1[m_dist[neighbour]].erase(neighbour);
                if(dict1[m_dist[neighbour]].empty()){
                    dict1.erase(m_dist[neighbour]);
                }
                m_dist[neighbour] = m_dist[curr] + calcGPSDistance(points[curr].first, points[curr].second, points[neighbour].first, points[neighbour].second);
                dict1[m_dist[neighbour]].insert(neighbour);
                pars1[neighbour] = curr;
            }
        }
        dict1[currDist].erase(curr);
        if(dict1[currDist].empty()){
            dict1.erase(currDist);
        }
    }
    vector<unsigned long long int> path;
    for (unsigned long long int i = curr; i != start; i = pars1[i])
    {
        path.push_back(i);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}
vector<unsigned long long int> MultithreadDoubleAStar(){
//    used.clear();
//    needToFinish = false;

    thread ModifiedAStar1(ModifiedAStar, 0);
    thread ModifiedAStar2(ModifiedAStar, 1);
    {
        std::unique_lock<std::mutex> lk(cv_mtx);
        cv.wait(lk, []{ return needToFinish.load(); });
    }
    if(ModifiedAStar1.joinable()){
        ModifiedAStar1.join();
    }
    if(ModifiedAStar2.joinable()){
        ModifiedAStar2.join();
    }
    vector<unsigned long long int> path;
    for(unsigned long long int pt = buffPoint; pt != start; pt = get<2>(pars[0][pt])){// Прыгаем по соседям из pars прямого A*-а пока не дойдем до старта
        path.push_back(pt);
    }
    path.push_back(start);// Добавляем старт в путь
    reverse(path.begin(), path.end());// Переворачиваем путь, потому что мы шли в обратном порядке
    for(unsigned long long int pt = get<2>(pars[1][buffPoint]); pt != finish; pt = get<2>(pars[1][pt])){// Прыгаем по соседям из pars обратного A*-а пока не дойдем до старта
        path.push_back(pt);
    }
    path.push_back(finish);// Добавляем финиш в путь
    return path;
}
using namespace rapidjson;
void buildGraph(Document& data, unordered_map<pair<double, double>, unordered_set<pair<double, double>, pair_hash>, pair_hash>& adj) {
    if (!data.HasMember("features") || !data["features"].IsArray()) {
        std::cerr << "Invalid JSON structure: missing features array" << std::endl;
        return;
    }
    const Value& features = data["features"];
    for (unsigned long long int i = 0; i < features.Size(); ++i) {
        const Value& feature = features[i];
        if (!feature.IsObject() ||
            !feature.HasMember("geometry") ||
            !feature["geometry"].IsObject() ||
            !feature["geometry"].HasMember("coordinates") ||
            !feature["geometry"]["coordinates"].IsArray()) {
            continue;
        }
        const Value& coordinates = feature["geometry"]["coordinates"];
        for (SizeType y = 1; y < coordinates.Size(); ++y) {
            const Value& point = coordinates[y];
            const Value& lastPoint = coordinates[y - 1];
            if (!point.IsArray() || point.Size() < 2 ||
                !point[0].IsNumber() || !point[1].IsNumber()) {
                continue;
            }
            adj[make_pair(point[0].GetDouble(), point[1].GetDouble())].insert(make_pair(lastPoint[0].GetDouble(), lastPoint[1].GetDouble()));
            adj[make_pair(lastPoint[0].GetDouble(), lastPoint[1].GetDouble())].insert(make_pair(point[0].GetDouble(), point[1].GetDouble()));
        }
    }
}
string pathToJson(std::vector<unsigned long long int>& path) {
    Document jsonPath(kObjectType);
    auto& allocator = jsonPath.GetAllocator();
    jsonPath.AddMember("type", "FeatureCollection", allocator);
    Value features(kArrayType);
    Value feature(kObjectType);
    feature.AddMember("type", "Feature", allocator);
    Value properties(kObjectType);
    properties.AddMember("@id", "way/33803251", allocator);
    properties.AddMember("highway", "residential", allocator);
    properties.AddMember("name", "X Corps Boulevard", allocator);
    feature.AddMember("properties", properties, allocator);
    Value geometry(kObjectType);
    Value coordinates(kArrayType);
    for (auto& point : path) {
        Value pointCoords(kArrayType);
        pointCoords.PushBack(points[point].first, allocator);
        pointCoords.PushBack(points[point].second, allocator);
        coordinates.PushBack(pointCoords, allocator);
    }
    geometry.AddMember("type", "LineString", allocator);
    geometry.AddMember("coordinates", coordinates, allocator);
    feature.AddMember("geometry", geometry, allocator);
    features.PushBack(feature, allocator);
    jsonPath.AddMember("features", features, allocator);
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);
    jsonPath.Accept(writer);
    return buffer.GetString();
}
void codePoints(){
    unsigned long long int cnt = 0;
    for(auto pr: adjListCoords){
        points[cnt] = pr.first;
        pts.push_back(pr.first);
        Indices[pr.first] = cnt;
        ++cnt;
    }
    for(auto pr: adjListCoords){
        for(auto pr1: pr.second){
            adjList[Indices[pr.first]].insert(Indices[pr1]);
        }
    }
}
unsigned long long int linearFindNearestPoint(double latitude, double longitude){
    double minDist = numeric_limits<double>::max();
    unsigned long long int nearestPoint;
    for(auto point: points){
        if(calcGPSDistance(latitude, longitude, point.second.second, point.second.first) < minDist){
            minDist = calcGPSDistance(latitude, longitude, point.second.second, point.second.first);
            nearestPoint = point.first;
        }
    }
    return nearestPoint;
}
double getLength(vector<unsigned long long int>& path){
    double sum = 0;
    for(int i = 1; i < path.size(); ++i){
        sum += calcGPSDistance(points[path[i - 1]].first, points[path[i - 1]].second, points[path[i]].first, points[path[i]].second);
    }
    return sum;
}
bool isFirstItem;
bool comparePairsFst(const pair<double, double>& a, const pair<double, double>& b){
    return a.first < b.first;
}
bool comparePairsSnd(const pair<double, double>& a, const pair<double, double>& b){
    return a.second < b.second;
}
struct pairsComparator {
    bool operator()(pair<double, double> a, pair<double, double> b) const
    {
        return a.second < b.second;
    }
};
map<double, vector<pair<double, double>>> splitTresholds;
map<double, map<double, vector<pair<double, double>>>> res;
unsigned long long int findNearestPoint(double longitude, double latitude){
    double minDist = numeric_limits<double>::max();
    pair<double, double> nearestPoint;
    try{
        auto iter = res.lower_bound(latitude);
        vector<pair<double, double>> curr = res[iter->first].lower_bound(longitude)->second;
        for(auto point: curr){
            if(calcGPSDistance(longitude, latitude, point.second, point.first) < minDist){
                minDist = calcGPSDistance(longitude, latitude, point.second, point.first);
                nearestPoint = point;
            }
        }
        if(iter != (res.begin()++)){
            auto before = (iter--);
            vector<pair<double, double>> bef = res[before->first].lower_bound(longitude)->second;
            for(auto point: bef){
                if(calcGPSDistance(longitude, latitude, point.second, point.first) < minDist){
                    minDist = calcGPSDistance(longitude, latitude, point.second, point.first);
                    nearestPoint = point;
                }
            }
        }
        if(iter != (res.end()--)){
            auto after = (res.lower_bound(latitude)++);
            vector<pair<double, double>> aft = res[after->first].lower_bound(longitude)->second;
            for(auto point: aft){
                if(calcGPSDistance(longitude, latitude, point.second, point.first) < minDist){
                    minDist = calcGPSDistance(longitude, latitude, point.second, point.first);
                    nearestPoint = point;
                }
            }
        }
    }catch(...) {// если мы последние в словаре
        auto after = (res.end()--);
        vector<pair<double, double>> aft = res[after->first].lower_bound(longitude)->second;
        for(auto point: aft){
            if(calcGPSDistance(longitude, latitude, point.second, point.first) < minDist){
                minDist = calcGPSDistance(longitude, latitude, point.second, point.first);
                nearestPoint = point;
            }
        }
        if(after != (res.begin()++)){
            auto before = (after--);
            vector<pair<double, double>> bef = res[before->first].lower_bound(longitude)->second;
            for(auto point: bef){
                if(calcGPSDistance(longitude, latitude, point.second, point.first) < minDist){
                    minDist = calcGPSDistance(longitude, latitude, point.second, point.first);
                    nearestPoint = point;
                }
            }
        }
    }
    return Indices[nearestPoint];
}
void splitGraph(int sideSize){
    sort(pts.begin(), pts.end(), comparePairsFst);
    int elementsInContainer = pts.size() / sqrt(sideSize);
    int cnt = 0;
    vector<pair<double, double>> buff;
    pair<double, double> lastPoint;
    for(pair<double, double>& point: pts){
        if(cnt != 0 && !(cnt % elementsInContainer) && cnt < sideSize * elementsInContainer){
            splitTresholds[lastPoint.first] = {buff};
            buff.clear();
        }
        buff.push_back(point);
        lastPoint = point;
        ++cnt;
    }
    auto iter = --splitTresholds.end();
    for(pair<double, double>& pr: buff){
        splitTresholds[iter->first].push_back(pr);
    }
    elementsInContainer /= sqrt(sideSize);
    for(auto& pr: splitTresholds){
        map<double, vector<pair<double, double>>> dict;
        sort(pr.second.begin(), pr.second.end(), comparePairsSnd);
        buff.clear();
        cnt = 0;
        for(pair<double, double>& point: pr.second){
            if(cnt != 0 && !(cnt % elementsInContainer) && cnt < sideSize * elementsInContainer){
                dict[lastPoint.second] = {buff};
                buff.clear();
            }
            buff.push_back(point);
            lastPoint = point;
            ++cnt;
        }
        auto iter = --dict.end();
        for(pair<double, double>& pr: buff){
            dict[iter->first].push_back(pr);
        }
        res[pr.first] = {dict};
    }
}
void printConnectivity(){
    vector<int> sums(30);
    for(auto pair:adjList){
        ++sums[pair.second.size()];
        //cout << pair.second.size() << '\n';
    }
    for(int cnt: sums){
        cout << cnt << '\n';
    }
}
void scan(){
    double minFst = numeric_limits<double>::max();
    double maxFst = numeric_limits<double>::min();
    double minSnd = numeric_limits<double>::max();
    double maxSnd = numeric_limits<double>::min();
    for(auto pr: points){
        if(pr.second.first < minFst){
            minFst = pr.second.first;
        }
        if(pr.second.first > maxFst){
            maxFst = pr.second.first;
        }
        if(pr.second.second < minSnd){
            minSnd = pr.second.second;
        }
        if(pr.second.second > maxSnd){
            maxSnd = pr.second.second;
        }
    }
    cout<<endl;
}
class Rand_double
{
public:
    Rand_double(double low, double high)
            : r([=]() {
        static std::random_device rd;
        static std::default_random_engine engine(rd());
        std::uniform_real_distribution<double> dist(low, high);
        return dist(engine);
    }) {}

    double operator()() { return r(); }

private:
    std::function<double()> r;
};
Rand_double rdFst{55.55, 55.90};  // Широта Москвы
Rand_double rdSnd{37.30, 37.85};  // Долгота Москвы
pair<unsigned long long int, unsigned long long int> generatePoints(double distance, double threshold){
    pair<double, double> p1 = make_pair(rdFst(), rdSnd());
    pair<double, double> p2 = make_pair(rdFst(), rdSnd());
    double dist = calcGPSDistance(p1.first, p1.second, p2.first, p2.second);
    while(abs(distance - dist) > threshold || linearFindNearestPoint(p1.first, p1.second) == linearFindNearestPoint(p2.first, p2.second)){
        p1 = make_pair(rdFst(), rdSnd());
        p2 = make_pair(rdFst(), rdSnd());
        //cout << linearFindNearestPoint(p1.first, p1.second) << '\n' << linearFindNearestPoint(p2.first, p2.second) << '\n';;
        dist = calcGPSDistance(p1.first, p1.second, p2.first, p2.second);
    }
    return make_pair(linearFindNearestPoint(p1.first, p1.second), linearFindNearestPoint(p2.first, p2.second));
}
void Test(int amountOfStages, int threshold, int testsInStage, int step){
    pair<unsigned long long int, unsigned long long int> buff;
    unsigned long long int p1, p2;
    vector<function<vector<unsigned long long int>()>> algorithms = {Dejkstra, AStar, DoubleAStar, MultithreadDoubleAStar, DoubleAStar, MultithreadDoubleAStar};
    vector<string> names = {"Дейкстра", "A*", "2A*", "Multithread 2A*", "2Дейкстра", "Multithread Дейкстра"};
    vector<unordered_map<int, unsigned long long int>> Res(algorithms.size());
    unsigned long long int time;
    cout << "Testing started\n";
    for(int dist = step; dist < (amountOfStages - 1) * step; dist += step){
        for(int y = 0; y < testsInStage; ++y){
            buff = generatePoints(dist, threshold);
            start = buff.first;
            finish = buff.second;
            for(int algorithm = 0; algorithm < algorithms.size(); ++algorithm){
                cout << "Distance: " << dist << " Test: " << y + 1 << " Algorithm: " << names[algorithm] << '\n';
                isDejkstra = false;
                if(algorithm == 4 || algorithm == 5){
                    isDejkstra = true;
                }
                pars[0].clear();
                dict[0].clear();
                pars[1].clear();
                dict[1].clear();
                pars1.clear();
                used.clear();
                needToFinish = false;
                time = clock();
                algorithms[algorithm]();
                Res[algorithm][dist] += clock() - time;
            }
        }
    }
    cout << "Testing finished\n";
    for(int algorithm = 0; algorithm < algorithms.size(); ++algorithm){
        cout << names[algorithm] << ":\n";
        for(auto pr: Res[algorithm]){
            cout << "Distance: " << pr.first << " Time: " << pr.second << '\n';
        }
    }
}
int main()
{
    std::ifstream file("/Users/vladimir/CLionProjects/2A*/moscow_roads.geojson");
    rapidjson::IStreamWrapper isw(file);
    rapidjson::Document doc;
    doc.ParseStream(isw);
    buildGraph(doc, adjListCoords);
    codePoints();
    Test(68, 500, 100, 1000);
    return 0;
}
