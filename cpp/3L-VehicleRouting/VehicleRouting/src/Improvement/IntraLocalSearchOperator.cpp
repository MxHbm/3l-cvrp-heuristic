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
                                    BaseLoadingChecker* loadingChecker,
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
                                BaseLoadingChecker* loadingChecker,
                                Collections::IdVector& route,
                                std::vector<IntraMove>& moves){

    if (moves.size() == 0)
    {
        return std::nullopt;
    }

    std::ranges::sort(moves, [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
    });

    auto set =  loadingChecker->MakeBitset(instance->Nodes.size(), route);

    const auto& container = instance->Vehicles.front().Containers.front();

    for (const auto& move: moves)
    {

        ChangeRoute(route, std::get<1>(move), std::get<2>(move));
        
        auto selectedItems = InterfaceConversions::SelectItems(route, instance->Nodes, false);
        if (loadingChecker->CompleteCheck(container, set, route, selectedItems, mType))
        {
            return std::get<0>(move);
        }
        
        //Change routes back if it was not feasible! 
        //TODO Dont forget to delete 
        RevertRoute(route, std::get<1>(move), std::get<2>(move));
    }

    return std::nullopt;


};

}
}