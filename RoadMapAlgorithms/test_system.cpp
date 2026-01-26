#include "test_system.h"
#include "algorithms/astar.h"
#include "algorithms/dijkstra.h"
#include "algorithms/bidirectional_astar.h"
#include "algorithms/multithread_bidirectional_astar.h"
#include "utils/graph.h"
#include "utils/distance.h"
#include <iostream>
#include <functional>
#include <random>
#include <vector>
#include <map>

bool isDejkstra = false;

namespace TestSystem {
    /// Класс для генерации случайных чисел с плавающей точкой
    class Rand_double
    {
    public:
        Rand_double(double low, double high): r([=]() {
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
    
    /// Линейный поиск ближайшей точки графа к данной точке
    unsigned long long int linearFindNearestPoint(double latitude, double longitude){
        double minDist = std::numeric_limits<double>::max();
        unsigned long long int nearestPoint = 0;
        for(auto point: Graph::points){
            if(Distance::calcGPSDistance(latitude, longitude, 
                                         point.second.second, point.second.first) < minDist){
                minDist = Distance::calcGPSDistance(latitude, longitude, 
                                                   point.second.second, point.second.first);
                nearestPoint = point.first;
            }
        }
        return nearestPoint;
    }
    
    /// Генератор точек на заданном расстоянии (distance) с погрешностью threshold
    std::pair<unsigned long long int, unsigned long long int> generatePoints(double distance, double threshold){
        std::pair<double, double> p1 = std::make_pair(rdFst(), rdSnd());
        std::pair<double, double> p2 = std::make_pair(rdFst(), rdSnd());
        double dist = Distance::calcGPSDistance(p1.first, p1.second, p2.first, p2.second);
        
        // Генерируем точки, пока не получим пару на нужном расстоянии и с разными ближайшими вершинами графа
        while(abs(distance - dist) > threshold || 
              linearFindNearestPoint(p1.first, p1.second) == linearFindNearestPoint(p2.first, p2.second)){
            p1 = std::make_pair(rdFst(), rdSnd());
            p2 = std::make_pair(rdFst(), rdSnd());
            dist = Distance::calcGPSDistance(p1.first, p1.second, p2.first, p2.second);
        }
        return std::make_pair(linearFindNearestPoint(p1.first, p1.second), 
                              linearFindNearestPoint(p2.first, p2.second));
    }
    
    /// Тестирующая система для алгоритмов
    void Test(int amountOfStages, int threshold, int testsInStage, int step){
        std::pair<unsigned long long int, unsigned long long int> buff;
        unsigned long long int p1, p2;
        
        // Вектор алгоритмов для тестирования
        std::vector<std::function<std::vector<unsigned long long int>()>> algorithms = {
            Dejkstra, AStar, DoubleAStar, MultithreadDoubleAStar, DoubleAStar, MultithreadDoubleAStar
        };
        std::vector<std::string> names = {
            "Дейкстра", "A*", "2A*", "Multithread 2A*", "2Дейкстра", "Multithread Дейкстра"
        };
        
        // Результаты тестирования для каждого алгоритма
        std::vector<std::map<int, unsigned long long int>> Res(algorithms.size());
        unsigned long long int time;
        
        std::cout << "Testing started\n";
        
        // Тестируем на разных расстояниях
        for(int dist = step; dist < (amountOfStages - 1) * step; dist += step){
            // Для каждого расстояния выполняем несколько тестов
            for(int y = 0; y < testsInStage; ++y){
                buff = generatePoints(dist, threshold);
                Graph::setStartFinish(buff.first, buff.second);
                
                // Тестируем каждый алгоритм
                for(int algorithm = 0; algorithm < algorithms.size(); ++algorithm){
                    std::cout << "Distance: " << dist << " Test: " << y + 1 
                              << " Algorithm: " << names[algorithm] << '\n';
                    
                    isDejkstra = false;
                    if(algorithm == 4 || algorithm == 5){ // Если это 2Дейкстра или Multithread Дейкстра, зануляем эвристику
                        isDejkstra = true;
                    }
                    
                    ClearMultithreadDoubleAStarData();
                    
                    time = clock(); // Начинаем замер времени
                    algorithms[algorithm](); // Запускаем алгоритм
                    Res[algorithm][dist] += clock() - time; // Фиксируем результат
                }
            }
        }
        
        // Выводим результаты тестирования
        std::cout << "Testing finished\n";
        for(int algorithm = 0; algorithm < algorithms.size(); ++algorithm){
            std::cout << names[algorithm] << ":\n";
            for(auto pr: Res[algorithm]){
                std::cout << "Distance: " << pr.first << " Time: " << pr.second << '\n';
            }
            std::cout << std::endl;
        }
    }
}