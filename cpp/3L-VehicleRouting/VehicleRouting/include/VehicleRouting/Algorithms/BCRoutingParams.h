#pragma once

#include "ContainerLoading/ProblemParameters.h"

#include <unordered_map>

// NOLINTBEGIN(readability-magic-numbers)

namespace VehicleRouting
{
namespace Algorithms
{
using namespace ContainerLoading;
using namespace ContainerLoading::Algorithms;


struct MIPSolverParams
{
  public:
    int Threads = 8;
    double Seed = 100;
    int EnableLazyConstraints = 1;
    int DisablePreCrush = 1;
    int CutGeneration = -1;
    int NumericFocus = 0;
    double TimeLimit = 12.0 * 3600.0;
    int MaxSolutions = std::numeric_limits<int>::max();

    MIPSolverParams() = default;
};

struct IteratedLocalSearchParams
{
  public:
    enum class StartSolutionType
    {
        None = 0,
        ModifiedSavings
    };

    enum class CallType
    {
        None,
        Exact,
        ExactLimit,
        ILS,
        Constructive
    };

    bool RunILS = false;
    bool RunLS = true;
    bool ActivateIntraRouteImprovement = false;
    unsigned int IntraRouteFullEnumThreshold = 0;

    bool ActivateSetPartitioningHeuristic = true;
    unsigned int SetPartitioningHeuristicThreshold = 20;
    StartSolutionType StartSolution = StartSolutionType::ModifiedSavings;

    std::unordered_map<CallType, double> TimeLimits = {
        {CallType::Exact, std::numeric_limits<double>::max()},
        {CallType::ExactLimit, 1.0},
        {CallType::ILS, 120.0},
        {CallType::Constructive, 10.0}
    };

    bool ActivateHeuristic = false;
    bool ActivateMemoryManagement = false;
    bool TrackIncrementalFeasibilityProperty = false;
};

class InputParameters
{
  public:
    MIPSolverParams MIPSolver;
    IteratedLocalSearchParams IteratedLocalSearch;
    ContainerLoadingParams ContainerLoading;

    void SetLoadingFlags() { ContainerLoading.LoadingProblem.SetFlags(); };

    [[nodiscard]] double DetermineMaxRuntime(IteratedLocalSearchParams::CallType callType,
                                             double residualTime = std::numeric_limits<double>::max()) const
    {
        return std::min(IteratedLocalSearch.TimeLimits.at(callType), residualTime);
    }

    [[nodiscard]] bool IsExact(IteratedLocalSearchParams::CallType callType) const
    {
        return callType == IteratedLocalSearchParams::CallType::Exact;
    }
};

}
}

// NOLINTEND(readability-magic-numbers)