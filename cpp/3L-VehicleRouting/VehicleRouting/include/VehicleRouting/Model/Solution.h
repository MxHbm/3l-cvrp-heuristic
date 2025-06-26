#pragma once

#include "Algorithms/BCRoutingParams.h"
#include "Algorithms/Evaluation.h"
#include "Helper/Timer.h"

#include "Model/Instance.h"
#include "Node.h"
#include "Vehicle.h"

#include <ranges>
#include <string>

namespace VehicleRouting
{
using namespace Algorithms;

namespace Model
{

// https://stackoverflow.com/questions/14589417/can-an-enum-class-be-converted-to-the-underlying-type
template <typename E> constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}
// https://stackoverflow.com/questions/69762598/what-are-commonly-used-ways-to-iterate-over-an-enum-class-in-c
constexpr inline auto enum_range = [](auto front, auto back)
{
    return std::views::iota(to_integral(front), to_integral(back) + 1)
           | std::views::transform([](auto e) { return decltype(front)(e); });
};


class Tour
{
  public:
    Node Depot;
    Model::Vehicle Vehicle;
    std::vector<Node> Route;

    Tour() = default;
    Tour(const Node& depot, const Model::Vehicle& vehicle, std::vector<Node>& route)
    : Depot(depot), Vehicle(vehicle), Route(route)
    {
    }

    Tour(const Node& depot, const Model::Vehicle& vehicle, std::vector<Node>&& route)
    : Depot(depot), Vehicle(vehicle), Route(route)
    {
    }

    std::string Print() const
    {
        std::string sequence;
        sequence.append(" ").append(std::to_string(0)).append("-");
        for (const auto& node: Route)
        {
            sequence.append(std::to_string(node.InternId)).append("-");
        }

        sequence.append(std::to_string(0)).append(" ");

        return sequence;
    };

    std::string PrintRoute() const
    {
        std::string sequence;
        for (const auto& node: Route)
        {
            sequence.append(std::to_string(node.InternId)).append(" ");
        }

        return sequence;
    };
};

class SolverStatistics
{
  public:
    double Runtime = -1.0;
    double Gap = -1.0;
    double NodeCount = -1;
    double SimplexIterationCount = -1;
    size_t DeletedArcs = 0;
    size_t InfeasibleTailPathStart = 0;
    Helper::Timer Timer;

    SolverStatistics(double runtime,
                     double gap,
                     double nodeCount,
                     double iterCount,
                     Helper::Timer& timer,
                     size_t deletedArcs,
                     size_t infTailPathStart)
    : Runtime(runtime),
      Gap(gap),
      NodeCount(nodeCount),
      SimplexIterationCount(iterCount),
      DeletedArcs(deletedArcs),
      InfeasibleTailPathStart(infTailPathStart),
      Timer(timer)
    {
    }
};

class Solution
{
  public:
    double Costs = 0.0;

    size_t NumberOfRoutes = 0;
    size_t LowerBoundVehicles = 0;

    std::vector<Tour> Tours;

    Solution() = default;

    void DetermineCosts(Instance* instance)
    {
        Costs = 0;
        for (const auto& tour: Tours)
        {
            Costs += Evaluator::CalculateRouteCosts(instance, tour.Route);
        }
    };
};

class SolutionFile
{
  public:
    VehicleRouting::InputParameters InputParameters;
    Model::SolverStatistics SolverStatistics;
    Model::Solution Solution;

    SolutionFile(VehicleRouting::InputParameters& inputParameters,
                 Model::SolverStatistics& statistics,
                 Model::Solution& solution)
    : InputParameters(inputParameters), SolverStatistics(statistics), Solution(solution)
    {
    }
};

}
}