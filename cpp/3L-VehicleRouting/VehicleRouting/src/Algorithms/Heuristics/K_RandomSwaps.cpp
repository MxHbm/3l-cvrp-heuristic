#include "Algorithms/Heuristics/K_RandomSwaps.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <unordered_set>
#include <algorithm>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
using namespace ContainerLoading;

void K_RandomSwaps::Run(const Instance* const instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    std::vector<Route>& routes,
                    std::mt19937& RNG)
{
    int succesful_swaps = 0;
    if (routes.size() < 2)
    {
        return;
    }

    //Preinitialize values
    const double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);
    const auto& container = instance->Vehicles.front().Containers.front();

    // Implement unordered_set as tabu list

    while(succesful_swaps < inputParameters.IteratedLocalSearch.K_RandomSwaps){

        // Implement unordered_set as tabu list
        //Update DetermineMovesTo give only one Move, where routes are different! 
        // Only impirotant taht weight and volume restrictions are applied! 
        // Copy code from GetBestMove here and update found swaps if swaps are feasible!

        auto move = DetermineMoves(instance, routes, RNG);

        if(!move){
            break;
        }

        bool controlFlag = true;
        
        ChangeRoutes(routes, *move);

        if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            UpdateRouteVolumeWeight(routes, *move);
            ++succesful_swaps;
            continue;
        }

        for(const auto& route_index : {std::get<0>(*move), std::get<1>(*move)})
        {
            auto& route = routes[route_index];
            auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);
            
            // If lifo is disabled, feasibility of route is independent from actual sequence
            // -> move is always feasible if route is feasible
            if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route.Sequence))
            {
                continue;
            }

            auto selectedItems = InterfaceConversions::SelectItems(route.Sequence, instance->Nodes, false);
            auto status = loadingChecker->HeuristicCompleteCheck(container, set, route.Sequence, selectedItems, maxRuntime);

            if (status != LoadingStatus::FeasOpt)
            {
               controlFlag = false;
               break;
            }

        }
        // Change routes back if not feasible!
        if (!controlFlag)
        {
            ChangeRoutes(routes, *move);
            continue;
        }

        UpdateRouteVolumeWeight(routes, *move);
        ++succesful_swaps;
    }  
}

std::optional<K_InterSwapMove> K_RandomSwaps::DetermineMoves(const Instance* const instance,
                                              const std::vector<Route>& routes,
                                              std::mt19937& RNG)
{


    std::uniform_int_distribution<size_t> route_dist(0, routes.size() - 1);
    int limit_route_draws = 10;
    int route_draws = 0;
    const auto container_weight_limit = instance->Vehicles.front().Containers.front().WeightLimit;
    const auto container_volume_limit = instance->Vehicles.front().Containers.front().Volume;

    while(route_draws < limit_route_draws){
        size_t route_it_i = route_dist(RNG);
        size_t route_it_k = route_dist(RNG); 

        while(route_it_i == route_it_k){
            route_it_i = route_dist(RNG);
            route_it_k = route_dist(RNG); 
        }

        const auto& route_i = routes[route_it_i];
        const auto& route_k = routes[route_it_k];
        const auto res_weight_route_i = container_weight_limit - route_i.TotalWeight;
        const auto res_volume_route_i = container_volume_limit - route_i.TotalVolume;
        const auto res_weight_route_k = container_weight_limit - route_k.TotalWeight;
        const auto res_volume_route_k = container_volume_limit - route_k.TotalVolume;

        std::uniform_int_distribution<size_t> route_i_dist(0, route_i.Sequence.size() - 1);
        std::uniform_int_distribution<size_t> route_k_dist(0, route_k.Sequence.size() - 1);

        int attempts = 0;

        //TODO check if this can be compared! 
        while(attempts <= 0.1 * route_i.Sequence.size() * route_k.Sequence.size()){

            const auto node_i = route_i_dist(RNG);
            const auto node_k = route_k_dist(RNG);
            const auto weight_item_delta = instance->Nodes[route_i.Sequence[node_i]].TotalWeight - instance->Nodes[route_k.Sequence[node_k]].TotalWeight;
            const auto volume_item_delta = instance->Nodes[route_i.Sequence[node_i]].TotalVolume - instance->Nodes[route_k.Sequence[node_k]].TotalVolume;

        
        if(res_weight_route_i + weight_item_delta >= 0 && res_weight_route_k - weight_item_delta >= 0)
            if(res_volume_route_i + volume_item_delta >= 0 && res_volume_route_k - volume_item_delta >= 0){
                return std::make_tuple(route_it_i, route_it_k, node_i, node_k, weight_item_delta, volume_item_delta);
        }
            
            ++attempts;
        }
        ++route_draws;
    }
    return std::nullopt;
}


void K_RandomSwaps::UpdateRouteVolumeWeight(std::vector<Route>& routes, const K_InterSwapMove& move){

    const auto item_delta = std::get<4>(move);
    const auto volume_delta = std::get<5>(move);

    auto& route_i = routes[std::get<0>(move)];
    auto& route_k = routes[std::get<1>(move)];

    route_i.TotalWeight -= item_delta;
    route_k.TotalWeight += item_delta;
    route_i.TotalVolume -= volume_delta;
    route_k.TotalVolume += volume_delta;
}

void K_RandomSwaps::ChangeRoutes(std::vector<Route>& routes, const K_InterSwapMove& move)
{

    auto node_i = std::get<2>(move);
    auto node_k = std::get<3>(move);

    auto& route_i = routes[std::get<0>(move)];
    auto& route_k = routes[std::get<1>(move)];

    std::swap(route_i.Sequence[node_i], route_k.Sequence[node_k]);
    
}
}
}
}
}