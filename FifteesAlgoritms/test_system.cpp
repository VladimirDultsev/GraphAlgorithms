#include "test_system.h"

#include "test_system.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <functional>

#include "algorithms/astar.h"
#include "algorithms/bfs.h"
#include "algorithms/bidirectional_astar.h"
#include "algorithms/bidirectional_bfs.h"
#include "algorithms/multithread_bidirectional_astar.h"
#include "algorithms/multithread_bidirectional_bfs.h"
#include "utils/fiftees_generator.h"
#include "utils/graph.h"

TestSystemSettings::TestSystemSettings(unsigned long long int threshold,
                                       unsigned char levelsAmount,
                                       unsigned char fifteesOnLevel,
                                       unsigned char swapsToLevel,
                                       unsigned char testSessionsAmount,
                                       std::string& resultsFilePath,
                                       bool needToSave)
    : threshold(threshold),
      levelsAmount(levelsAmount),
      fifteesOnLevel(fifteesOnLevel),
      swapsToLevel(swapsToLevel),
      testSessionsAmount(testSessionsAmount),
      resultsFilePath(resultsFilePath),
      needToSave(needToSave){}

unsigned long long int TestSystemSettings::SecondsToClocks(double seconds)
{
    return static_cast<unsigned long long int> (seconds * CLOCKS_PER_SEC);
}

TestSystem::TestSystem(const TestSystemSettings& settings): settings(settings) {}

TestSystemSettings TestSystem::getTestSystemSettings() {
    return settings;
}

void TestSystem::StartTest()
{
    std::vector<std::vector<std::vector<unsigned short>>> DataBase(settings.levelsAmount,
        std::vector<std::vector<unsigned short>>(settings.fifteesOnLevel, std::vector<unsigned short>(16)));
    std::vector<std::function<std::vector<std::vector<unsigned short>>(std::vector<unsigned short>&)>> algorithms = {
        BFS,
        DoubleBFS,
        Multithread_2BFS,
        AStar,
        DoubleAStar,
        Multithread_2AStar
    };
    std::vector<std::string> names = {
        "BFS",
        "2BFS",
        "Multithread 2BFS",
        "A*",
        "2A*",
        "Multithread 2A*"
    };
    std::vector<std::vector<unsigned long long int>> res(algorithms.size(),
                                                         std::vector<unsigned long long int>(settings.levelsAmount));
    std::vector<unsigned short> field;
    for (int r = 0; r < settings.testSessionsAmount; ++r) {
        for (int i = 0; i < settings.levelsAmount; ++i) {
            for (int y = 0; y < settings.fifteesOnLevel; ++y) {
                Graph::longToField(fifteesGenerator((i + 1) * settings.swapsToLevel), field);
                DataBase[i][y] = field;
            }
        }
        std::cout << "Generation finished\n";
        for (int i = 0; i < algorithms.size(); ++i) {
            std::cout << "START TESTING: " << names[i] << '\n';
            for (int y = 0; y < settings.levelsAmount; ++y) {
                for (int u = 0; u < settings.fifteesOnLevel; ++u) {
                    std::cout << "Session " << r + 1 << " from " << (short)settings.testSessionsAmount
                         << ", Now testing: " << names[i]
                         << ", Swaps on level: " << (y + 1) * settings.swapsToLevel
                         << ", Test num: " << u + 1 << " from " << (short)settings.fifteesOnLevel << '\n';
                    Graph::error = false;
                    Graph::Start = clock();
                    algorithms[i](DataBase[y][u]);
                    res[i][y] += clock() - Graph::Start;
                    if (Graph::error) {
                        break;
                    }
                }
                if (Graph::error) {
                    break;
                }
            }
        }
    }

    std::cout << "Testing finished\n";
    for (int i = 0; i < algorithms.size(); ++i) {
        std::cout << "Algorithm: " << names[i] << '\n';
        for (int y = 0; y < settings.levelsAmount; ++y) {
            std::cout << (y + 1) * settings.swapsToLevel << " "
                 << res[i][y] / (settings.fifteesOnLevel * settings.testSessionsAmount) << '\n';
        }
    }

    if (settings.needToSave)
    {
        saveToCSV(settings.resultsFilePath, res, names);
    }
}

void TestSystem::saveToCSV(const std::string& filename,
                          const std::vector<std::vector<unsigned long long int>>& results,
                          const std::vector<std::string>& algorithmNames){
    std::ofstream csvFile(filename);
    if (!csvFile.is_open()) {
        std::cerr << "Error opening CSV file " << filename << "\n";
        return;
    }
    csvFile << "Swaps";
    for (const auto& name : algorithmNames) {
        csvFile << "," << name;
    }
    csvFile << "\n";
    for (unsigned int level = 0; level < settings.levelsAmount; ++level) {
        int swaps = (level + 1) * settings.swapsToLevel;
        csvFile << swaps;
        for (unsigned int alg = 0; alg < algorithmNames.size(); ++alg) {
            unsigned long long avgTime = 0;
            if (alg < results.size() && level < results[alg].size()) {
                avgTime = results[alg][level] / (settings.fifteesOnLevel * settings.testSessionsAmount);
            }
            csvFile << "," << avgTime;
        }
        csvFile << "\n";
    }
    csvFile.close();
    std::cout << "CSV results saved to: " << filename << std::endl;
}
