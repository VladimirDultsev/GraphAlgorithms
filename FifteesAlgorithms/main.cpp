#include "test_system.h"
#include <iostream>

int main() {
    TestSystemSettings settings;

    // Можно настроить параметры (или оставить значения по умолчанию)
    double thresholdInSeconds = 15; // Лимит времени выполнения алгоритма, по достижении которого он принудительно завершается
    settings.threshold = settings.SecondsToClocks(thresholdInSeconds);
    settings.levelsAmount = 15;           // Количество уровней сложности
    settings.fifteesOnLevel = 3;          // Количество тестов на каждом уровне
    settings.swapsToLevel = 1;            // Шаг увеличения количества перестановок между уровнями
    settings.testSessionsAmount = 10;     // Количество сессий тестирования
    settings.resultsFilePath = "../test_results/results.csv";  // Файл для сохранения результатов
    settings.needToSave = true;           // Сохранять ли результаты в CSV файл

    // Создание тестовой системы с настройками
    TestSystem testSystem(settings);

    // Запуск тестирования
    std::cout << "\nНачало тестирования...\n";
    testSystem.StartTest();
    std::cout << "Тестирование завершено!\n";
    return 0;
}