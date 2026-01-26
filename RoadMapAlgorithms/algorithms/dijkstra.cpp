#include "dijkstra.h"
#include "../utils/graph.h"
#include "../utils/distance.h"
#include <map>
#include <unordered_set>
#include <limits>
#include <algorithm>

using namespace Graph;
/// Алгоритм Дейкстры
std::vector<unsigned long long int> Dejkstra(){
    std::unordered_map<unsigned long long int, unsigned long long int> pars1; // родитель
    std::vector<double> m_dist(Graph::adjList.size(), std::numeric_limits<double>::max());
    std::map<double, std::unordered_set<unsigned long long int>> dict1;
    
    dict1[0].insert(start);
    m_dist[start] = 0; // Массив, в котором для каждой вершины хранится наименьшее расстояние, за которое мы смогли дойти до неё
    
    unsigned long long int curr; // Рассматриваемая вершина
    double currDist; // Расстояние, вершины на котором мы сейчас рассматриваем
    
    while(!dict1.empty()){ // Пока в словаре есть точки
        currDist = dict1.begin()->first; // Расстояние, вершины на котором мы сейчас рассматриваем - это наименьшее расстояние, присутствующее в словаре
        curr = *dict1.begin()->second.begin();
        
        if(curr == finish){ // Если вершина, которую мы достали - это финиш, завершаемся
            //cout << "solved\n";
            break;
        }
        
        for(unsigned long long int neighbour: Graph::adjList[curr]){ // Перебираем соседей точки
            double edgeDist = Distance::calcGPSDistance(
                Graph::points[curr].first, Graph::points[curr].second,
                Graph::points[neighbour].first, Graph::points[neighbour].second);
            double newDist = m_dist[curr] + edgeDist;
            
            if(m_dist[neighbour] == std::numeric_limits<double>::max()){ // Если мы впервые встречаем данную вершину, то обязательно добавляем её
                // Расстояние до неё - это минимальное расстояние, за которое мы дошли до данной вершины + длина ребра до неё
                m_dist[neighbour] = newDist;
                dict1[m_dist[neighbour]].insert(neighbour); // Добавляем точку в словарь под приоритетом в виде расстояния, за которое мы дошли до неё
                pars1[neighbour] = curr; // Теперь наша вершина - родитель найденной нами вершины
            }
            // Если мы уже встречали эту вершину, но дошли до неё за меньшее расстояние, чем раньше
            else if(newDist < m_dist[neighbour]){
                dict1[m_dist[neighbour]].erase(neighbour); // Удаляем эту вершину из словаря под старым приоритетом
                if(dict1[m_dist[neighbour]].empty()){ // Если точек на таком расстоянии больше нет, удаляем этот ключ из словаря
                    dict1.erase(m_dist[neighbour]);
                }
                // Новое расстояние до этой вершины - это минимальное расстояние, за которое мы дошли до данной вершины + длина ребра до неё
                m_dist[neighbour] = newDist;
                dict1[m_dist[neighbour]].insert(neighbour); // Добавляем точку в словарь под приоритетом в виде расстояния, за которое мы дошли до неё
                pars1[neighbour] = curr; // Теперь наша вершина - родитель найденной нами вершины
            }
        }
        
        dict1[currDist].erase(curr); // Удаляем текущую точку из словаря, так ка мы её уже рассмотрели
        if(dict1[currDist].empty()){ // Если точек на таком расстоянии больше нет, удаляем этот ключ из словаря
            dict1.erase(currDist);
        }
    }
    
    // Восстановление пути
    std::vector<unsigned long long int> path;
    for (unsigned long long int i = curr; i != start; i = pars1[i]) // Прыгаем по родителям, пока не дойдём до старта
    {
        path.push_back(i);
    }
    path.push_back(start); // Добавляем старт в путь
    std::reverse(path.begin(), path.end()); // Переворачиваем полученный массив, потому что мы шли в обратном порядке
    return path;
}