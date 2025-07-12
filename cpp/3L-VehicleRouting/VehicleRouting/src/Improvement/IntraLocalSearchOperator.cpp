#pragma once

#include "Improvement/IntraLocalSearchOperator.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

void IntraLocalSearchOperator::Run(const Instance* instance,
                const InputParameters& inputParameters,
                LoadingChecker* loadingChecker,
                Solution& currentSolution)
{
   for(auto& route : currentSolution.Routes){

        if (route.Sequence.size() < 2)
        {
            continue;
        }

        while(true){

            auto moves = DetermineMoves(instance, route.Sequence);
            auto savings = GetBestMove(instance, inputParameters, loadingChecker, route.Sequence, moves);

            if(!savings){
                break;
            }else{
                currentSolution.Costs += *savings;
            }
        }
    }
       
};

std::optional<double> IntraLocalSearchOperator::GetBestMove(const Instance* instance,
                                const InputParameters& inputParameters,
                                LoadingChecker* loadingChecker,
                                Collections::IdVector& route,
                                std::vector<IntraMove>& moves){

    if (moves.size() == 0)
    {
        return std::nullopt;
    }

    std::ranges::sort(moves, [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
    });

    auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

    const auto& container = instance->Vehicles.front().Containers.front();
    double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);

    for (const auto& move: moves)
    {

        ChangeRoute(route, std::get<1>(move), std::get<2>(move));

        if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            return std::get<0>(move);
        }

        // If lifo is disabled, feasibility of route is independent from actual sequence
        // -> move is always feasible if route is feasible
        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route))
        {
            return std::get<0>(move);
        }

        auto selectedItems = InterfaceConversions::SelectItems(route, instance->Nodes, false);
        auto status = loadingChecker->HeuristicCompleteCheck(container, set, route, selectedItems, maxRuntime);

        if (status == LoadingStatus::FeasOpt)
        {
            return std::get<0>(move);
        }
        
        //Change routes back if it was not feasible! 
        ChangeRoute(route, std::get<1>(move), std::get<2>(move));
    }

    return std::nullopt;


};

}
}