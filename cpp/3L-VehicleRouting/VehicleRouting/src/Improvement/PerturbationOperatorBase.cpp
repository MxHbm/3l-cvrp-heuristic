#include "Improvement/PerturbationOperatorBase.h"

namespace VehicleRouting {
namespace Improvement {

void PerturbationOperatorBase::Run(const Model::Instance*            instance,
        const InputParameters&            params,
        ContainerLoading::LoadingChecker* loadingChecker,
        Model::Solution&                  solution,
        std::mt19937&                     rng)
{
    int succesful_moves = 0;

    std::vector<Route>& routes = solution.Routes;

    if (routes.size() < 2)
    {
        return;
    }

    //Preinitialize values
    const auto& container = instance->Vehicles.front().Containers.front();

    // Implement unordered_set as tabu list

    while(succesful_moves < params.IteratedLocalSearch.K_RandomMoves){

        // Implement unordered_set as tabu list
        //Update DetermineMovesTo give only one Move, where routes are different! 
        // Only impirotant taht weight and volume restrictions are applied! 
        // Copy code from GetBestMove here and update found swaps if swaps are feasible!

        auto move = DetermineMoves(instance, routes, rng);

        if(!move){
            break;
        }

        bool controlFlag = true;

        ChangeRoutes(routes, *move);

        if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            UpdateRouteVolumeWeight(routes, *move);
            ++succesful_moves;
            continue;
        }

        for(const auto& route_index : {std::get<1>(*move), std::get<2>(*move)})
        {
            auto& route = routes[route_index];

            if(route.Sequence.empty()){
                continue;
            }
            // If lifo is disabled, feasibility of route is independent from actual sequence
            // -> move is always feasible if route is feasible
            
            if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route.Sequence))
            {
                continue;
            }
            
            auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);
            auto selectedItems = Algorithms::InterfaceConversions::SelectItems(route.Sequence, instance->Nodes, false);

            if(params.ContainerLoading.classifierParams.UseClassifier){    
                if(params.ContainerLoading.classifierParams.SaveTensorData){

                    auto cpStatus = loadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                                        container,
                                                                        set,
                                                                        route.Sequence,
                                                                        selectedItems,
                                                                        false);

                    if(cpStatus == LoadingStatus::FeasOpt){

                        if(!loadingChecker->classifyWriteTensorData(selectedItems,route.Sequence,container,1)){
                            controlFlag = false;
                            break;
                        }

                    }else{

                        if(!loadingChecker->classifyWriteTensorData(selectedItems,route.Sequence,container,0)){
                            controlFlag = false;
                            break;
                        }
                    }
                
                }else{
                    if(!loadingChecker->classify(selectedItems,route.Sequence,container)) {
                        controlFlag = false;
                        break;
                    }
                }
            }else{
                if(!loadingChecker->CompleteCheck(container,
                                                    set,
                                                    route.Sequence,
                                                    selectedItems))                                            
                {
                    controlFlag = false;
                    break;
                }
        }

        }
        // Change routes back if not feasible!
        if (!controlFlag)
        {
            RevertChangeRoutes(routes, *move);
            continue;
        }

        UpdateRouteVolumeWeight(routes, *move);
        solution.Costs += std::get<0>(*move);
        ++succesful_moves;
    }  
};

void PerturbationOperatorBase::UpdateRouteVolumeWeight(std::vector<Route>& routes, const PerturbationMove& move){

    const auto item_delta = std::get<5>(move);
    const auto volume_delta = std::get<6>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    route_i.TotalWeight -= item_delta;
    route_k.TotalWeight += item_delta;
    route_i.TotalVolume -= volume_delta;
    route_k.TotalVolume += volume_delta;
};


}
}  // namespace
