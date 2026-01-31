#include <iostream>
#include <unordered_set>
#include <map>
#include <utility>
#include <vector>
#include <stack>
#include <queue>
#include <random>
#include <thread>
#include <mutex>
#include <fstream>
using namespace std;

vector<vector<unsigned short>> Purpose(2, vector<unsigned short>(16));
map<unsigned long long int, unordered_set<unsigned long long int>> dict[2];
unordered_map<unsigned long long int, tuple<double, unsigned long long int, unsigned long long int, bool>>pars[2];//эвристика, minDist (чистое расстояние без эвристики), родитель
vector<bool> vars(4, false);
vector<unsigned long long int> longs;
vector<pair<unsigned long long int, unsigned short>> Longs;
vector<unsigned short> Field(16), Finish = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0};
unsigned long long int Res, finish = 1311768467463790320, num, threshold, Start;
unsigned short amountOfVars, zeroPos, buffer1, buffer2, AStarIndex = 0;;//20 - 2.7 и 2
unordered_map<unsigned long long int, tuple<double, unsigned long long int, unsigned long long int, unsigned short>> Pars; //эвристика, minDist (чистое расстояние без эвристики), родитель, то, была ли точка пересчитана.
unordered_map<unsigned long long int, tuple<unsigned long long int, bool ,unsigned long long int>> PArs[2];//расстояние, была ли посещена и родитель
unordered_map<unsigned long long int, tuple<unsigned long long int, bool ,unsigned long long int>> PARs;//расстояние, была ли посещена и родитель
unordered_map<unsigned long long int, tuple<unsigned long long int, bool ,unsigned long long int>>pars_multithread[2];//расстояние, была ли посещена и родитель
unordered_set<unsigned long long int> used;
double delta, cnt;
bool error = false;
bool isDejkstra;
mutex cvMtx, mtx;
condition_variable cv;
atomic<bool> needToFinish{false};
unsigned long long int buffPoint, stPos;
unordered_map<unsigned long long int, tuple<double, unsigned long long int, unsigned long long int, bool>>pars1, pars2;
map<unsigned long long int, unordered_set<unsigned long long int>> dict1, dict2;
vector<vector<unsigned short>> fstPart, sndPart;

void printField(vector<unsigned short>& f)
{
    for (int i = 0; i < 16; i++)
    {
        if (i % 4 == 0)
        {
            cout << endl;
        }
        cout << f[i] << " ";
    }
    cout << endl;
}
class coding {
    void print(unsigned long long int l)
    {
        coding coding;
        coding.longToField(l);
        for (int i = 0; i < 16; i++)
        {
            if (i % 4 == 0)
            {
                cout << endl;
            }
            cout << Field[i] << " ";
        }
        cout << endl;
    }
public:
    coding()= default;
    void longToField(unsigned long long int num)
    {
        for (int i = 0; i < 16; i++)
        {
            Field[15 - i] = i == 0 ? (unsigned short)(num % 16) : (unsigned short)((num >> i * 4) % 16);
        }
    }
    unsigned long long int fieldToLong(vector<unsigned short>& f)
    {
        Res = 0;
        for (int i = 0; i < 16; i++)
        {
            Res = Res == 15 ? f[i] : Res | (unsigned long long int)f[i] << 60 - i * 4;
        }
        return Res;
    }
    unsigned long long int swap(unsigned long long int st, const unsigned short i, const unsigned short zeroPos)
    {
        num = (unsigned long long int)((((unsigned long long int)15<<4*(15 - i))&st)>>4*(15 - i));
        st -= (unsigned long long int)((((unsigned long long int)15<<4*(15 - i))&st));
        st += (unsigned long long int)(((unsigned long long int)num<<4*(15 - zeroPos)));
        return st;
    }
    void CalcVariants(unsigned long long int f)
    {
        zeroPos = 0;
        Longs.clear();
        longToField(f);
        for (unsigned short i = 0; i < 16; i++)
        {
            if((unsigned long long int)((((unsigned long long int)15<<4*i)&f)>>4*i) == 0)// если ноль - это ноль
            {
                zeroPos = 15 - i;
                break;
            }
        }
        if (zeroPos - 4 > 0)
        {
            Longs.push_back({swap(f, zeroPos - 4, zeroPos), zeroPos - 4});
        }
        if (zeroPos + 4 <= 15)
        {
            Longs.push_back({swap(f, zeroPos + 4, zeroPos), zeroPos + 4});
        }
        if (zeroPos != 0 && zeroPos != 4 && zeroPos != 8 && zeroPos != 12)
        {
            Longs.push_back({swap(f, zeroPos - 1, zeroPos), zeroPos - 1});
        }
        if (zeroPos != 3 && zeroPos != 7 && zeroPos != 11 && zeroPos != 15)
        {
            Longs.push_back({swap(f, zeroPos + 1, zeroPos), zeroPos + 1});
        }
    }
    double SmartHeuristic(unsigned long long int f)
    {
        longToField(f);
        cnt = 0;
        for (int i = 0; i < 16; ++i)
        {
            if (Field[i] != 0)
            {
                cnt += abs(i % 4 - (Field[i] - 1) % 4) + abs(i / 4 - (Field[i] - 1) / 4);
            }
        }
        delta = 0.4;
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
};

bool amountOfInversions(vector<unsigned short>& f)
{
    int amountOfInversions = 0;//она вызывается только один раз, так что ок
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16 - i; j++)
        {
            if (f[i + j] < f[i] && f[i] != 0)
            {
                amountOfInversions++;
            }
        }
    }
    if (amountOfInversions % 2 == 0)
    {
        return true;
    }
    return false;
}
unsigned long long int swap(unsigned long long int st, const unsigned short i, const unsigned short zeroPos)
{
    num = (unsigned long long int)((((unsigned long long int)15<<4*(15 - i))&st)>>4*(15 - i));
    st -= (unsigned long long int)((((unsigned long long int)15<<4*(15 - i))&st));
    st += (unsigned long long int)(((unsigned long long int)num<<4*(15 - zeroPos)));
    return st;
}
void CalcVariants(unsigned long long int f)
{
    amountOfVars = 0;
    zeroPos = 0;
    longs.clear();
    for (unsigned short i = 0; i < 16; i++)
    {
        if((unsigned long long int)((((unsigned long long int)15<<4*i)&f)>>4*i) == 0)// если ноль - это ноль
        {
            zeroPos = 15 - i;
            break;
        }
    }
    if (zeroPos - 4 > 0)
    {
        longs.push_back(swap(f, zeroPos - 4, zeroPos));
    }
    if (zeroPos + 4 <= 15)
    {
        longs.push_back(swap(f, zeroPos + 4, zeroPos));
    }
    if (zeroPos != 0 && zeroPos != 4 && zeroPos != 8 && zeroPos != 12)
    {
        longs.push_back(swap(f, zeroPos - 1, zeroPos));
    }
    if (zeroPos != 3 && zeroPos != 7 && zeroPos != 11 && zeroPos != 15)
    {
        longs.push_back(swap(f, zeroPos + 1, zeroPos));
    }
}
void CalcVariants(unsigned long long int f, vector<unsigned long long int>& ls)
{
    amountOfVars = 0;
    zeroPos = 0;
    ls.clear();
    for (unsigned short i = 0; i < 16; i++)
    {
        if((unsigned long long int)((((unsigned long long int)15<<4*i)&f)>>4*i) == 0)// если ноль - это ноль
        {
            zeroPos = 15 - i;
            break;
        }
    }
    if (zeroPos - 4 > 0)
    {
        ls.push_back(swap(f, zeroPos - 4, zeroPos));
    }
    if (zeroPos + 4 <= 15)
    {
        ls.push_back(swap(f, zeroPos + 4, zeroPos));
    }
    if (zeroPos != 0 && zeroPos != 4 && zeroPos != 8 && zeroPos != 12)
    {
        ls.push_back(swap(f, zeroPos - 1, zeroPos));
    }
    if (zeroPos != 3 && zeroPos != 7 && zeroPos != 11 && zeroPos != 15)
    {
        ls.push_back(swap(f, zeroPos + 1, zeroPos));
    }
}
unsigned long long int fieldToLong(const vector<unsigned short>& f)
{
    Res = 0;
    for (int i = 0; i < 16; i++)
    {
        Res |= (unsigned long long int)f[i] << (60 - i * 4);
    }
    return Res;
}
mutex longToFieldMutex;
void longToField(unsigned long long int num)
{
    {
        lock_guard<mutex> lock(longToFieldMutex);
        for (int i = 0; i < 16; i++)
        {
            Field[15 - i] = (unsigned short)((num >> i * 4) % 16);
        }
    }
}
unsigned short DoubleAStarHeuristic(unsigned long long int f)
{
    longToField(f);
    cnt = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (Field[i] != 0)
        {
            cnt += abs((i % 4) - (Purpose[AStarIndex][Field[i]] % 4)) + abs((i / 4) - (Purpose[AStarIndex][Field[i]] / 4));// /1.07
        }
    }
    return cnt;
}
mutex HeuristicMutex;
double MultithreadDoubleAStarHeuristic(unsigned long long int f, int ind)
{
    longToField(f);
    cnt = 0;
    for (int i = 0; i < 16; ++i)
    {
        if (Field[i] != 0)
        {
            {
                lock_guard<mutex> lock(HeuristicMutex);
                cnt += abs((i % 4) - (Purpose[ind][Field[i]] % 4)) + abs((i / 4) - (Purpose[ind][Field[i]] / 4));// /1.07
            }
        }
    }
    return cnt;
}
void calcPurposes(vector<unsigned short>& st)
{
    //0-й A* - прямой, 1-й A* - обратный
    Purpose[0] = {15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
    for(unsigned short i = 0; i < 16; ++i)
    {
        Purpose[1][st[i]] = i;
    }
}
void print(unsigned long long int l)
{
    longToField(l);
    for (int i = 0; i < 16; i++)
    {
        if (i % 4 == 0)
        {
            cout << endl;
        }
        cout << Field[i] << " ";
    }
    cout << endl;
}
vector<vector<unsigned short>> AStar(vector<unsigned short>& st)
{
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unsigned long long int n_dist, point = 0, lastDist = 0, start = fieldToLong(st);
    unordered_set<unsigned long long int> EmptySet;
    unsigned short useless;
    coding coding;
    AStarIndex = 0;
    pars[0].clear();
    pars[1].clear();
    dict[0].clear();
    dict[1].clear();
    Start = clock();
    pars[0][start] = make_tuple(coding.SmartHeuristic(start), 0, start, false);
    dict[0][get<0>(pars[0][start])].insert(start);
    while (!dict[0].empty())
    {
        if(clock() - Start > threshold)
        {
            cout << "Time limit error!\n";
            vector<vector<unsigned short>> usl;
            error = true;
            return usl;
        }
        point = *((dict[AStarIndex].begin())->second).begin();
        if (point == finish)
        {
            break;
        }
        CalcVariants(point);
        for (unsigned long long int neighbour : longs)
        {
            newNeighbourDist = get<1>(pars[AStarIndex][point]) + // Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
                               1;
            if (!pars[AStarIndex].count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = coding.SmartHeuristic(neighbour);// Считаем эвристику до него
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
        pars[AStarIndex][point] = make_tuple(get<0>(pars[AStarIndex][point]), get<1>(pars[AStarIndex][point]), get<2>(pars[AStarIndex][point]), true);
        n_dist = get<1>(pars[AStarIndex][point]) + get<0>(pars[AStarIndex][point]);
        if (dict[AStarIndex].find(n_dist) != dict[AStarIndex].end())
        {
            dict[AStarIndex][n_dist].erase(point);
        }
        if (dict[AStarIndex][n_dist].empty())
        {
            dict[AStarIndex].erase(n_dist);
        }
    }
//    if(clock() - Start > threshold)
//    {
//        cout << "Time limit error!\n";
//        vector<vector<unsigned short>> usl;
//        error = true;
//        return usl;
//    }
//    coding coding;
//    double finalDist, oldDist, heuristic, newNeighbourDist;
//    map<double, unordered_set<unsigned long long int>> Dict;
//    unsigned long long int n_dist, point = 0, lastDist = 0, start = coding.fieldToLong(st);
//    unsigned short useless;
//    unordered_set<unsigned long long int> EmptySet;
//    Pars.clear();
//    for (unsigned short i = 0; i < 16; i++)
//    {
//        if(st[i] == 0)// (unsigned long long int)((((unsigned long long int)15<<4*i)&start)>>4*i) == 0
//        {
//            zeroPos = i;
//            break;
//        }
//    }
//    Start = clock();
//    Pars[start] = make_tuple(coding.SmartHeuristic(start), 0, start, zeroPos);
//    Dict[get<0>(Pars[start])].insert(start);
//    while (!Dict.empty())
//    {
//        point = *((Dict.begin())->second).begin();
//        if (point == finish)
//        {
//            break;
//        }
//        coding.CalcVariants(point);
//        for (pair<unsigned long long int, unsigned short> neighbour : Longs)
//        {
//            newNeighbourDist = get<1>(Pars[point]) + // Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
//                               1;
//            if (Pars.find(neighbour.first) == Pars.end())
//            {
//                heuristic = coding.SmartHeuristic(neighbour.first);
//                Pars[neighbour.first] = make_tuple(useless, get<1>(Pars[point]) + 1, point, neighbour.second);
//                finalDist = newNeighbourDist + heuristic;// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
//                Dict[finalDist].insert(neighbour.first);
//
//
//
////                useless = coding.SmartHeuristic(l.first);
////                Pars[l.first] = make_tuple(useless, get<1>(Pars[point]) + 1, point, l.second);
////                lastDist = get<1>(Pars[point]) + 1 + useless;
////                if (Dict.find(lastDist) == Dict.end())
////                {
////                    Dict[lastDist] = EmptySet;
////                }
////                Dict[lastDist].insert(l.first);
//            }
//            else if (get<1>(Pars[point]) + 1 < get<1>(Pars[neighbour.first]))
//            {
//                oldDist = get<0>(Pars[neighbour.first]) + get<1>(Pars[neighbour.first]);// Считаем приоритет, под которым сосед был положен в словарь раньше
//                Dict[oldDist].erase(neighbour.first);// Удаляем соседа из словаря
//                if (Dict[oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
//                    Dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
//                }
//                get<1>(Pars[neighbour.first]) = newNeighbourDist;// Обновляем расстояние до соседа
//                get<2>(Pars[neighbour.first]) = point;// Теперь мы - родитель этого соседа
//                finalDist = newNeighbourDist + get<0>(Pars[neighbour.first]);// В словарь кладем точку под приоритетом в виде суммы расстояния до соседа и эвристики
//                Dict[finalDist].insert(neighbour.first);
//
////                lastDist = get<1>(Pars[l.first]) + get<0>(Pars[l.first]);
////                Pars[l.first] = make_tuple(get<0>(Pars[l.first]), get<1>(Pars[point]) + 1, point, get<2>(Pars[point]));
////                if (Dict.find(lastDist) == Dict.end())
////                {
////                    Dict[lastDist] = EmptySet;
////                }
////                Dict[lastDist].insert(l.first);
////                if (Dict.find(lastDist) != Dict.end())
////                {
////                    Dict[lastDist].erase(l.first);
////                }
////                if (Dict[lastDist].empty())
////                {
////                    Dict.erase(lastDist);
////                }
//            }
//        }
//        oldDist = get<0>(Pars[point]) + get<1>(Pars[point]);// Считаем приоритет, под которым наша точка лежала в словаре
//        Dict[oldDist].erase(point);// Удаляем точку из словаря
//        if (Dict[oldDist].empty()) {// Если в словаре больше нет точек с таким приоритетом
//            Dict[AStarIndex].erase(oldDist);// Удаляем этот ключ из словаря
//        }
//    }
    vector<vector<unsigned short>> lst;
    for (unsigned long long int i = point; i != start; i = get<2>(pars[AStarIndex][i]))
    {
        coding.longToField(i);
        lst.push_back(Field);
    }
    coding.longToField(start);
    lst.push_back(Field);
    reverse(lst.begin(), lst.end());
    return lst;
}
vector<vector<unsigned short>> DoubleAStar(vector<unsigned short>& st)
{
    Purpose[0].clear();
    Purpose[1].clear();
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unsigned long long int n_dist, point = 0, lastDist = 0, start = fieldToLong(st);
    unordered_set<unsigned long long int> EmptySet;
    unsigned short useless;
    calcPurposes(st);
    AStarIndex = 0;
    pars[0].clear();
    pars[1].clear();
    dict[0].clear();
    dict[1].clear();
    Start = clock();
    pars[0][start] = make_tuple(DoubleAStarHeuristic(start), 0, start, false);
    dict[0][get<0>(pars[0][start])].insert(start);
    AStarIndex = 1;
    pars[1][finish] = make_tuple(DoubleAStarHeuristic(finish), 0, finish, false);
    dict[1][get<0>(pars[1][finish])].insert(finish);
    while (!dict[0].empty() && !dict[1].empty())
    {
        if(clock() - Start > threshold)
        {
            cout << "Time limit error!\n";
            vector<vector<unsigned short>> usl;
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
        if (get<3>(pars[1 - AStarIndex][point]))
        {
            break;
        }
        if(AStarIndex == 0 && point == finish || AStarIndex == 1 && point == start)
        {
            break;
        }
        CalcVariants(point);
        for (unsigned long long int neighbour : longs)
        {
            newNeighbourDist = get<1>(pars[AStarIndex][point]) + // Расстояние, за которое мы дошли до этого соседа (сумма расстояния за которое мы дошли до нас + расстояние до соседа)
                               1;
            if (!pars[AStarIndex].count(neighbour))// Если до этого мы не встречали нашего соседа
            {
                heuristic = DoubleAStarHeuristic(neighbour);// Считаем эвристику до него
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
        pars[AStarIndex][point] = make_tuple(get<0>(pars[AStarIndex][point]), get<1>(pars[AStarIndex][point]), get<2>(pars[AStarIndex][point]), true);
        n_dist = get<1>(pars[AStarIndex][point]) + get<0>(pars[AStarIndex][point]);
        if (dict[AStarIndex].find(n_dist) != dict[AStarIndex].end())
        {
            dict[AStarIndex][n_dist].erase(point);
        }
        if (dict[AStarIndex][n_dist].empty())
        {
            dict[AStarIndex].erase(n_dist);
        }
    }
    vector<vector<unsigned short>> lst;
    for (unsigned long long int i = point; i != start; i = get<2>(pars[0][i]))
    {
        longToField(i);
        lst.push_back(Field);
    }
    reverse(lst.begin(), lst.end());
    for(unsigned long long int i = get<2>(pars[1][point]); i != finish; i = get<2>(pars[1][i]))
    {
        longToField(i);
        lst.push_back(Field);
    }
    lst.push_back(Field);
    return lst;
}
vector<vector<unsigned short>> DoubleBFS(vector<unsigned short>& st)
{
    unsigned long long int curr;
    unsigned short BFSIndex = 0;
    unsigned long long int start = fieldToLong(st);
    queue<unsigned long long int> q[2];
    PArs[0].clear();
    PArs[1].clear();
    Start = clock();
    q[0].push(start);
    PArs[0][start] = make_tuple(0, true, 0);
    q[1].push(finish);
    PArs[1][finish] = make_tuple(0, true, 0);
    while(!q[0].empty() && !q[1].empty())
    {
        if(clock() - Start > threshold)
        {
            cout << "Time limit error!\n";
            vector<vector<unsigned short>> usl;
            error = true;
            return usl;
        }
        BFSIndex = 1 - BFSIndex;
        curr = q[BFSIndex].front();
        if(get<1>(PArs[1-BFSIndex][curr]))
        {
            break;
        }
        if(!BFSIndex && curr == finish || BFSIndex && curr == start)
        {
            cout<<"error"<<'\n';
            break;
        }
        q[BFSIndex].pop();
        CalcVariants(curr);
        for(unsigned long long int l : longs)
        {
            if(!get<1>(PArs[BFSIndex][l]))
            {
                PArs[BFSIndex][l] = make_tuple(get<0>(pars[BFSIndex][curr]) + 1, true, curr);
                q[BFSIndex].push(l);
            }
        }
    }
    vector<vector<unsigned short>> lst;
    for(unsigned long long int point = curr; point != 0; point = get<2>(PArs[0][point])) {
        longToField(point);
        lst.push_back(Field);
    }
    reverse(lst.begin(), lst.end());
    for(unsigned long long int point = get<2>(PArs[1][curr]); point != 0; point = get<2>(PArs[1][point])) {
        longToField(point);
        lst.push_back(Field);
    }
    return lst;
}
vector<vector<unsigned short>> BFS(vector<unsigned short>& st)
{
    unsigned long long int curr;
    unsigned long long int start = fieldToLong(st);
    queue<unsigned long long int> q;
    PARs.clear();
    Start = clock();
    q.push(start);
    PARs[start] = make_tuple(0, true, 0);
    while(!q.empty())
    {
        if(clock() - Start > threshold)
        {
            cout << "Time limit error!\n";
            vector<vector<unsigned short>> usl;
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
        CalcVariants(curr);
        for(unsigned long long int l : longs)
        {
            if(!get<1>(PARs[l]))
            {
                PARs[l] = make_tuple(get<0>(PARs[curr]) + 1, true, curr);
                q.push(l);
            }
        }
    }
    vector<vector<unsigned short>> lst;
    for(unsigned long long int point = get<2>(PARs[finish]); point != 0; point = get<2>(PARs[point])) {
        longToField(point);
        lst.push_back(Field);
    }
    std::reverse(lst.begin(), lst.end());
    lst.push_back({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0});
    return lst;
}
void ModifiedBFS(int BFSIndex)
{
    unordered_map<unsigned long long int, tuple<unsigned long long int, bool ,unsigned long long int>> pars;
    unsigned long long int curr, start;
    vector<unsigned long long int> neighbours;
    if (!BFSIndex)
    {
        start = stPos;
    }
    else
    {
        start = finish;
    }
    queue<unsigned long long int> q;
    q.push(start);
    pars[start] = make_tuple(0, true, 0);
    while(!q.empty() && !needToFinish && !error)
    {
        if(clock() - Start > threshold)
        {
            mtx.lock();
            needToFinish = true;
            error = true;
            mtx.unlock();
            cv.notify_all();
            cout << "Time limit error!\n";
            break;
        }
        curr = q.front();
        unique_lock<mutex> lock(mtx, defer_lock);
        lock.lock();
        if((!BFSIndex && curr == finish) || (BFSIndex && curr == stPos) || used.count(curr))
        {
            if ((!BFSIndex && curr == finish) || (BFSIndex && curr == start))
            {
                cout << "Solved by one bfs\n";
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
            if(!get<1>(pars[l]))
            {
                pars[l] = make_tuple(get<0>(pars[curr]) + 1, true ,curr);
                q.push(l);
            }
        }
    }
    if (error)
    {
        return;
    }
    for(unsigned long long int pt = buffPoint; pt != start; pt = get<2>(pars[pt]))
    {
        longToField(pt);
        if (!BFSIndex)
        {
            fstPart.push_back(Field);
        }else
        {
            sndPart.push_back(Field);
        }
    }
}
vector<vector<unsigned short>> Multithread_2BFS(vector<unsigned short>& st){
    fstPart.clear();
    sndPart.clear();
    used.clear();
    needToFinish = false;
    error = false;
    stPos = fieldToLong(st);
    finish = fieldToLong(Finish);
    Start = clock();

    thread ModifiedBFS1(ModifiedBFS, 0);
    thread ModifiedBFS2(ModifiedBFS, 1);
    {
        unique_lock<mutex> lk(cvMtx);
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
    vector<vector<unsigned short>> path;
    path.insert(path.end(), fstPart.begin(), fstPart.end());
    path.push_back(st);
    reverse(path.begin(), path.end());
    path.insert(path.end(), sndPart.begin(), sndPart.end());
    path.push_back(Finish);

    return path;
}
void ModifiedAStar(int AStarIndex) {
    double finalDist, oldDist, heuristic, newNeighbourDist;
    unsigned long long int point = 0, start;
    double n_dist;
    unordered_map<unsigned long long int, tuple<double, double, unsigned long long int>> pars;
    map<double, unordered_set<unsigned long long int>> dict;
    unordered_set<unsigned long long int> localUsed;
    vector<unsigned long long int> neighbours;
    longToField(stPos);
    calcPurposes(Field);
    if (!AStarIndex) {
        start = stPos;
    } else {
        start = finish;
    }
    pars[start] = make_tuple(DoubleAStarHeuristic2(start, AStarIndex), 0, start);
    dict[get<0>(pars[start]) + get<1>(pars[start])].insert(start);
    while (!dict.empty() && !needToFinish && !error) {
        if (clock() - Start > threshold) {
            lock_guard<mutex> lock(mtx);
            needToFinish = true;
            error = true;
            cv.notify_all();
            cout << "Time limit error!\n";
            break;
        }
        point = *((dict.begin())->second).begin();
        if (point == 0)
        {
            cout << endl;
        }
        unique_lock<mutex> lock(mtx, defer_lock);
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
            if (neighbour == 0)
            {
                cout << endl;
            }
            newNeighbourDist = get<1>(pars[point]) + 1;
            if (!pars.count(neighbour)) {
                heuristic = DoubleAStarHeuristic2(neighbour, AStarIndex);
                pars[neighbour] = make_tuple(heuristic, newNeighbourDist, point);
                finalDist = newNeighbourDist + heuristic;
                dict[finalDist].insert(neighbour);
            } else if (newNeighbourDist < get<1>(pars[neighbour])) {
                oldDist = get<0>(pars[neighbour]) + get<1>(pars[neighbour]);
                if (dict.count(oldDist)) {
                    dict[oldDist].erase(neighbour);
                    if (dict[oldDist].empty()) {
                        dict.erase(oldDist);
                    }
                }
                get<1>(pars[neighbour]) = newNeighbourDist;
                get<2>(pars[neighbour]) = point;
                finalDist = newNeighbourDist + get<0>(pars[neighbour]);
                dict[finalDist].insert(neighbour);
            }
        }
        pars[point] = make_tuple(get<0>(pars[point]), get<1>(pars[point]), get<2>(pars[point]));
        n_dist = get<1>(pars[point]) + get<0>(pars[point]);
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
    if (!AStarIndex) {
        vector<unsigned long long int> tempPath;
        for (unsigned long long int pt = get<2>(pars[buffPoint]); pt != stPos; pt = get<2>(pars[pt])) {
            tempPath.push_back(pt);
        }
        tempPath.push_back(stPos);
        reverse(tempPath.begin(), tempPath.end());
        for (auto pt : tempPath) {
            longToField(pt);
            fstPart.push_back(Field);
        }
    } else {
        vector<unsigned long long int> tempPath;
        for (unsigned long long int pt = get<2>(pars[buffPoint]); pt != finish; pt = get<2>(pars[pt])) {
            tempPath.push_back(pt);
        }
        tempPath.push_back(finish);
        for (auto pt : tempPath) {
            longToField(pt);
            sndPart.push_back(Field);
        }
    }
}

vector<vector<unsigned short>> Multithread_2AStar(vector<unsigned short>& st) {
    fstPart.clear();
    sndPart.clear();
    used.clear();
    needToFinish = false;
    error = false;
    stPos = fieldToLong(st);
    Start = clock();
    calcPurposes(st);
    thread ModifiedAstar1(ModifiedAStar, 0);
    thread ModifiedAstar2(ModifiedAStar, 1);
    {
        unique_lock<mutex> lk(cvMtx);
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
    vector<vector<unsigned short>> path;
    path.insert(path.end(), fstPart.begin(), fstPart.end());
    longToField(buffPoint);
    path.insert(path.end(), sndPart.begin(), sndPart.end());
    for (auto& f: path)
    {
        print(fieldToLong(f));
    }
    return path;
}

unsigned long long int fifteesGenerator(int targetDistance) {
    random_device rd;
    mt19937 rng(rd());

    unordered_map<unsigned long long int, int> distances;
    queue<unsigned long long int> q;
    vector<unsigned long long int> statesAtTargetDistance;

    distances[finish] = 0;
    q.push(finish);

    while (!q.empty()) {
        unsigned long long int current = q.front();
        q.pop();
        int currentDist = distances[current];
        if (currentDist == targetDistance) {
            statesAtTargetDistance.push_back(current);
            if (statesAtTargetDistance.size() > 1000) break;
            continue;
        }
        if (currentDist > targetDistance) continue;
        CalcVariants(current);
        for (auto& neighbor : longs) {
            if (distances.find(neighbor) == distances.end()) {
                distances[neighbor] = currentDist + 1;
                q.push(neighbor);
            }
        }
    }

    if (!statesAtTargetDistance.empty()) {
        uniform_int_distribution<unsigned long> dist(0, statesAtTargetDistance.size() - 1);
        unsigned long long int selected = statesAtTargetDistance[dist(rng)];
        longToField(selected);
        std::vector<std::vector<unsigned short>> solutionPath = DoubleAStar(Field);
        std::cout << "Сгенерировано состояние с длиной решения: " << solutionPath.size() << '\n';
        return selected;
    }
    cout << "Не найдено состояний с расстоянием " << targetDistance << '\n';
    return finish;
}

int main() {
    threshold = 15000000;
    unsigned char levelsAmount = 15, fifteesOnLevel = 3, swapsToLevel = 1, testSessionsAmount = 10;
    vector<vector<vector<unsigned short>>> DataBase(levelsAmount,vector<vector<unsigned short>>(fifteesOnLevel, vector<unsigned short>(16)));
    vector<function<vector<vector<unsigned short>>(vector<unsigned short>&)>> algorithms = {
        BFS,
        DoubleBFS,
        Multithread_2BFS,
        AStar,
        DoubleAStar,
        Multithread_2AStar
    };
    vector<string> names = {
        "BFS",
        "2BFS",
        "Multithread 2BFS",
        "A*",
        "2A*",
        "Multithread 2A*"
    };
    vector<vector<unsigned long long int>> res(algorithms.size(),
        vector<unsigned long long int>(levelsAmount));
    for (int r = 0; r < testSessionsAmount; ++r) {
        for (int i = 0; i < levelsAmount; ++i) {
            for (int y = 0; y < fifteesOnLevel; ++y) {
                longToField(fifteesGenerator((i + 1) * swapsToLevel));
                DataBase[i][y] = Field;
            }
        }
        cout << "Generation finished\n";
        for (int i = 0; i < algorithms.size(); ++i) {
            cout << "START TESTING: " << names[i] << '\n';
            for (int y = 0; y < levelsAmount; ++y) {
                for (int u = 0; u < fifteesOnLevel; ++u) {
                    cout << "Session " << r + 1 << " from " << (short)testSessionsAmount
                         << ", Now testing: " << names[i]
                         << ", Swaps on level: " << (y + 1) * swapsToLevel
                         << ", Test num: " << u + 1 << " from " << (short)fifteesOnLevel << '\n';
                    error = false;
                    needToFinish = false;
                    Start = clock();
                    algorithms[i](DataBase[y][u]);
                    res[i][y] += clock() - Start;
                    if (error) {
                        break;
                    }
                }
                if (error) {
                    break;
                }
            }
        }
    }

    cout << "Testing finished\n";
    for (int i = 0; i < algorithms.size(); ++i) {
        cout << "Algorithm: " << names[i] << '\n';
        for (int y = 0; y < levelsAmount; ++y) {
            cout << (y + 1) * swapsToLevel << " "
                 << res[i][y] / (fifteesOnLevel * testSessionsAmount) << '\n';
        }
    }
    return 0;
}