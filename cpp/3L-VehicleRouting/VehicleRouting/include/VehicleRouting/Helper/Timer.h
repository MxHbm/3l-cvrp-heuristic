#pragma once

#include <cassert>
#include <chrono>
#include <functional>
#include <iostream>

namespace VehicleRouting
{
namespace Helper
{
class Timer
{
  public:
    std::chrono::duration<double> InfeasibleArcs;
    std::chrono::duration<double> LowerBoundVehicles;
    std::chrono::duration<double> StartSolution;
    std::chrono::duration<double> MetaHeuristic;
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::time_point<std::chrono::system_clock> overall_start;
    
    double elapsed;
    
    Timer() = default;

    inline void startOverallTime(){
        overall_start = std::chrono::system_clock::now();
    }

    inline void calculateElapsedTime(){
        elapsed = std::chrono::duration<double>(std::chrono::system_clock::now() - overall_start).count();  // result in seconds as double (e.g., 16.2541)
    }

    inline double getElapsedTime() const{
        return elapsed;
    }

    void calculateMetaHeuristicTime(){
        MetaHeuristic = std::chrono::system_clock::now() - overall_start - StartSolution;
    }

    void calculateStartSolutionTime(){
        StartSolution = std::chrono::system_clock::now() - overall_start;
    }

    void Print() const
    {
        std::cout << "Infeasible Arcs: " << InfeasibleArcs.count() << "\n";
        std::cout << "Lower bound: " << LowerBoundVehicles.count() << "\n";
        std::cout << "Start solution: " << StartSolution.count() << "\n";
        std::cout << "Branch-and-cut: " << MetaHeuristic.count() << "\n";
    }
};

// https://stackoverflow.com/questions/2808398/easily-measure-elapsed-time
template <class TimeT = std::chrono::milliseconds, class ClockT = std::chrono::steady_clock>

class FunctionTimer
{
    using timep_t = typename ClockT::time_point;

  public:
    void start()
    {
        mEnd = timep_t{};
        mStart = ClockT::now();
    }

    void end() { mEnd = ClockT::now(); }

    [[nodiscard]] uint64_t elapsed() const { return std::chrono::duration_cast<TimeT>(mEnd - mStart).count(); }

  private:
    timep_t mStart = ClockT::now();
    timep_t mEnd = {};

    template <class TT = TimeT> TT duration() const
    {
        assert((mEnd != timep_t{}) && "toc before reporting");
        return std::chrono::duration_cast<TT>(mEnd - mStart);
    }
};

template <class TimeT = std::chrono::microseconds, class ClockT = std::chrono::steady_clock> struct measure
{
    template <class F, class... Args> static auto duration(F&& func, Args&&... args)
    {
        auto start = ClockT::now();
        std::invoke(std::forward<F>(func), std::forward<Args>(args)...);
        return std::chrono::duration_cast<TimeT>(ClockT::now() - start);
    }
    template <class F, class... Args> static auto durationWithReturn(F&& func, Args&&... args)
    {
        auto start = ClockT::now();
        decltype(auto) res{std::invoke(std::forward<F>(func), std::forward<Args>(args)...)};
        return std::make_pair(res, std::chrono::duration_cast<TimeT>(ClockT::now() - start));
    }
};

}
}