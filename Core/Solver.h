#ifndef CORE_SOLVER_H
#define CORE_SOLVER_H

#include <vector>
#include <chrono>
#include <ctime>
#include <atomic>

namespace core {
    /// Метрики выполнения - время в секундах, процессорное время, наличие таймаута
    struct TimeMetrics {
        double realTime = 0.0;
        double cpuTime = 0.0;
        bool timeout = false;
    };

    template <typename GraphType>
    class Solver {
    public:
        using StateType = typename GraphType::State;

        Solver() : timeLimit_(0.0), error_(false) {}
        virtual ~Solver() = default;

        /// Возвращает кратчайший путь между start и goal
        virtual std::vector<StateType> solve(const StateType& start, const StateType& goal) = 0;

        /// Устанавливает лимит времени выполнения алгоритма
        void setTimeLimit(double seconds) {
            timeLimit_ = seconds;
        }

        /// Возвращает установленный сейчас лимит времени
        double getTimeLimit() const {
            return timeLimit_;
        }

        /// Возвращает наличие ошибки выполнения
        bool hasError() const {
            return error_.load();
        }

        /// Сбрасывает ошибку выполнения
        void resetError() {
            error_ = false;
        }

        /// Возвращает метрики выполнения
        [[nodiscard]] const TimeMetrics& getMetrics() const {
            return metrics_;
        }

        /// Сбрасывает метрики выполнения
        void resetMetrics() {
            metrics_ = TimeMetrics{};
            error_ = false;
        }

    protected:
        /// Начинает замер времени работы
        void startTimers() {
            startRealTime_ = std::chrono::steady_clock::now();
            startCpuTime_ = std::clock();
        }

        /// Обновляет метрики выполнения
        void updateMetrics() {
            if (timeLimit_ > 0 && metrics_.timeout) {
                return;
            }

            metrics_.realTime = std::chrono::duration<double>(std::chrono::steady_clock::now()
                    - startRealTime_).count();

            metrics_.cpuTime = static_cast<double>(std::clock() - startCpuTime_);
        }

        /// Проверяет наличие таймаута выполнения
        bool checkTimeout() {
            if (timeLimit_ <= 0.0) return false;

            double currTime = std::chrono::duration<double>(std::chrono::steady_clock::now()
                    - startRealTime_).count();

            if (currTime > timeLimit_) {
                metrics_.timeout = true;
                error_ = true;
                return true;
            }
            return false;
        }

        /// Возвращает текущее процессорное время
        [[nodiscard]] double getCurrentCpuTime() const;

        /// Возвращает текущее реальное время
        [[nodiscard]] double getCurrentRealTime() const;

        double timeLimit_;
        std::atomic<bool> error_;
        TimeMetrics metrics_;

    private:
        std::chrono::steady_clock::time_point startRealTime_;
        std::clock_t startCpuTime_;
    };



    template<typename GraphType>
    double Solver<GraphType>::getCurrentRealTime() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - startRealTime_).count();
    }

    template<typename GraphType>
    double Solver<GraphType>::getCurrentCpuTime() const {
        return static_cast<double>(std::clock() - startCpuTime_);
    }

}
#endif