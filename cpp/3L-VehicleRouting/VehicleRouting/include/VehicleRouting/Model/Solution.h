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

class SolutionTracker
{
  public:
    std::map<double, double> CurrentSolution;
    std::map<double, double> BestSolution;
    std::map<double, double> NoImprStatistics;
    int iterations{0};
    int rejections{0};
    int NoImpr{0};
    int RoundsWithNoImpr{0};


    //Default constructor
    SolutionTracker(){};

    void UpdateCurrSolution(const double runtime, const double bound)
    {
        CurrentSolution.insert({runtime, bound});
    }

    void UpdateBestSolution(const double runtime, const double bound)
    {
        BestSolution.insert({runtime, bound});
    }

    void UpdateBothSolutions(const double runtime, const double bound){

        BestSolution.insert({runtime, bound});
        CurrentSolution.insert({runtime, bound});
    }


};

class SolverStatistics
{
  public:
    size_t ILSIterationCount = 0;
    size_t rejectionCount = 0;
    size_t DeletedArcs = 0;
    size_t InfeasibleTailPathStart = 0;
    Helper::Timer Timer;
    SolutionTracker solutionTracker;

    SolverStatistics(Helper::Timer& timer,
                     SolutionTracker& solTracker,
                     size_t deletedArcs,
                     size_t infTailPathStart)
    : ILSIterationCount(solTracker.iterations),
      rejectionCount (solTracker.rejections),
      DeletedArcs(deletedArcs),
      InfeasibleTailPathStart(infTailPathStart),
      Timer(timer),
      solutionTracker(solTracker)
    {
    }
};


class Solution
{
  public: 
    double Costs = 0.0; 
    size_t NumberOfRoutes = 0;

    std::vector<Route> Routes; 

    Solution() = default;

    void DetermineCosts(Instance* instance)
    {
        Costs = 0;
        for (const auto& route: Routes)
        {
            Costs += Evaluator::CalculateRouteCosts(instance, route.Sequence);
        }
    };

    void DeterminWeightsVolumes(Instance* instance)
    {
        for (auto& route: Routes)
        {   
            route.TotalVolume = 0;
            route.TotalWeight = 0;

            for (const auto& node_Id: route.Sequence){
                
                route.TotalVolume += instance->Nodes[node_Id].TotalVolume;
                route.TotalWeight += instance->Nodes[node_Id].TotalWeight;
            }
        }
    };
};

class OutputSolution
{
  public:
    double Costs = 0.0;

    size_t NumberOfRoutes = 0;
    size_t LowerBoundVehicles = 0;

    std::vector<Tour> Tours;

    OutputSolution() = default;
    OutputSolution(const std::vector<Tour>& tours, double costs, size_t numberOfRoutes, size_t lowerBoundVehicles)
    : Tours(tours), Costs(costs), NumberOfRoutes(numberOfRoutes), LowerBoundVehicles(lowerBoundVehicles)
    {
    }

    OutputSolution(const Solution& solution,Instance* instance)
    : Costs(solution.Costs), NumberOfRoutes(solution.Routes.size()), LowerBoundVehicles(instance->LowerBoundVehicles)
    {
      
      Tours.reserve(NumberOfRoutes);
      int vehicleId = 0;

      for (const auto& route: solution.Routes)
      { 

          std::vector<Node> sequence;
          sequence.reserve(route.Sequence.size());
          for (const auto intern_id: route.Sequence)
          {
              sequence.emplace_back(instance->Nodes[intern_id]);
          }

          Vehicle& vehicle = instance->Vehicles[vehicleId];
          Tours.emplace_back(instance->Nodes[instance->DepotIndex], vehicle, std::move(sequence));
          ++vehicleId;
      }
  };

    void DetermineCosts(Instance* instance)
    {
        Costs = 0;
        for (const auto& tour: Tours)
        {
            Costs += Evaluator::CalculateRouteCosts(instance, tour.Route);
        }
    };
};

//TODO if mCurrentSolutionArcs is needed, then add values to output solution format! 
/*
    for (const auto& route: mCurrentSolution.Routes)
    {
        mCurrentSolutionArcs.emplace_back(1.0, mInstance->GetDepotId(), route.Sequence.front());

        for (size_t iNode = 0; iNode < route.Sequence.size() - 1; ++iNode)
        {
            auto nodeA = route.Sequence[iNode];
            auto nodeB = route.Sequence[iNode + 1];
            mCurrentSolutionArcs.emplace_back(1.0, nodeA, nodeB);
        }

        mCurrentSolutionArcs.emplace_back(1.0, route.Sequence.back(), mInstance->GetDepotId());
    }
*/

class SolutionFile
{
  public:
    VehicleRouting::InputParameters InputParameters;
    Model::SolverStatistics SolverStatistics;
    Model::OutputSolution OutputSolution;

    SolutionFile(VehicleRouting::InputParameters& inputParameters,
                 Model::SolverStatistics& statistics,
                 Model::OutputSolution& outputSolution)
    : InputParameters(inputParameters), SolverStatistics(statistics), OutputSolution(outputSolution)
    {
    }
};

}
}