#ifndef TEST_SYSTEM_H
#define TEST_SYSTEM_H
#include <string>
#include <vector>

struct TestSystemSettings
{
    unsigned long long int threshold = 15000000;
    unsigned char levelsAmount = 15;
    unsigned char fifteesOnLevel = 3;
    unsigned char swapsToLevel = 1;
    unsigned char testSessionsAmount = 10;
    std::string resultsFilePath = "test_results/results.csv";
    bool needToSave = true;

public:
    TestSystemSettings() = default;
    TestSystemSettings(const TestSystemSettings&) = default;
    TestSystemSettings(unsigned long long int threshold, 
                      unsigned char levelsAmount, 
                      unsigned char fifteesOnLevel,
                      unsigned char swapsToLevel, 
                      unsigned char testSessionsAmount,
                      std::string& resultsFilePath,
                      bool needToSave);
    static unsigned long long int SecondsToClocks(double seconds);
};

struct TestSystem
{
    TestSystemSettings settings;
    
    TestSystem() = default;
    TestSystem(const TestSystemSettings& settings);
    
    TestSystemSettings getTestSystemSettings();

    void saveToCSV(const std::string& filename,
                          const std::vector<std::vector<unsigned long long int>>& results,
                          const std::vector<std::string>& algorithmNames);

    void StartTest();
};

#endif