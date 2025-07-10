#include "Improvement/InterSwap.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "CommonBasics/Helper/ModelServices.h"

#include <algorithm>

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;

void InterSwap::Run(const Instance* const instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    Solution& currentSolution)
{

    std::vector<Route>& routes = currentSolution.Routes;

    if (routes.size() < 2)
    {
        return;
    }

    while(true){

        auto moves = DetermineMoves(instance, routes);
        auto savings = GetBestMove(instance, inputParameters, loadingChecker, routes, moves);

        if(!savings){
            break;
        }else{
            currentSolution.Costs += *savings;
        }
    }

    return;
       
}

std::vector<InterSwapMove> InterSwap::DetermineMoves(const Instance* const instance,
                                                     const std::vector<Route>& routes)
{

    std::vector<InterSwapMove> moves{};
    const auto container_weight_limit = instance->Vehicles.front().Containers.front().WeightLimit;
    const auto container_volume_limit = instance->Vehicles.front().Containers.front().Volume;

    for (size_t route_it_i = 0; route_it_i < routes.size() - 1; ++route_it_i)
    {
        const auto& route_i = routes[route_it_i];
        const auto res_weight_route_i = container_weight_limit - route_i.TotalWeight;
        const auto res_volume_route_i = container_volume_limit - route_i.TotalVolume;


        for (size_t route_it_k = route_it_i + 1; route_it_k < routes.size(); ++route_it_k)
        {
            const auto& route_k = routes[route_it_k];
            const auto res_weight_route_k = container_weight_limit - route_k.TotalWeight;
            const auto res_volume_route_k = container_volume_limit - route_k.TotalVolume;

            for ( size_t node_i = 0; node_i < route_i.Sequence.size(); ++node_i)
            {
                const auto internId_node_i = route_i.Sequence[node_i];

                for (size_t node_k = 0; node_k < route_k.Sequence.size(); ++node_k)
                {
                    const auto internId_node_k = route_k.Sequence[node_k];
                    const auto weight_item_delta = instance->Nodes[internId_node_i ].TotalWeight - instance->Nodes[internId_node_k].TotalWeight;
                    const auto volume_item_delta = instance->Nodes[internId_node_i ].TotalVolume - instance->Nodes[internId_node_k].TotalVolume;

                    if(res_weight_route_i + weight_item_delta >= 0 && res_weight_route_k - weight_item_delta >= 0)
                        if(res_volume_route_i + volume_item_delta >= 0 && res_volume_route_k - volume_item_delta >= 0){

                            auto savings = Evaluator::CalculateInterSwapDelta(instance,
                                                                            route_i.Sequence,
                                                                            route_k.Sequence,
                                                                            node_i,
                                                                            node_k); 


                            if (savings < -1e-6)
                            {
                                moves.emplace_back(savings, route_it_i, route_it_k, node_i, node_k, weight_item_delta, volume_item_delta);
                            }
                    }
                }


            }
        }
    }

    return moves;
}


std::optional<double> InterSwap::GetBestMove(const Instance* const instance,
                            const InputParameters& inputParameters,
                            LoadingChecker* loadingChecker,
                            std::vector<Route>& routes,
                            std::vector<InterSwapMove>& moves)
{

    if (moves.size() == 0)
    {
        return std::nullopt;
    }

    std::ranges::sort(moves, [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
    });
    
    //TODO - Create Bitset for all the two routes!
    //auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

    //Initiate variables before loop
    const double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);
    const auto& container = instance->Vehicles.front().Containers.front();

    for (const auto& move: moves)
    {
        bool controlFlag = true;
        

        if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            UpdateRouteVolumeWeight(routes, move);
            return std::get<0>(move);
        }

        ChangeRoutes(routes, move);


        for(auto& route_index : {std::get<1>(move), std::get<2>(move)})
        {
            auto& route = routes[route_index];
            auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);
            
            // If lifo is disabled, feasibility of route is independent from actual sequence
            // -> move is always feasible if route is feasible
            if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route.Sequence))
            {
                UpdateRouteVolumeWeight(routes, move);
                return std::get<0>(move);
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
            ChangeRoutes(routes, move);
            continue;
        }

        UpdateRouteVolumeWeight(routes, move);
        return std::get<0>(move);
    }

    return std::nullopt;
}

void InterSwap::UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterSwapMove& move){

    const auto item_delta = std::get<5>(move);
    const auto volume_delta = std::get<6>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    route_i.TotalWeight -= item_delta;
    route_k.TotalWeight += item_delta;
    route_i.TotalVolume -= volume_delta;
    route_k.TotalVolume += volume_delta;
}


void InterSwap::ChangeRoutes(std::vector<Route>& routes, const InterSwapMove& move)
{

    auto node_i = std::get<3>(move);
    auto node_k = std::get<4>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    std::swap(route_i.Sequence[node_i], route_k.Sequence[node_k]);
    
}
}
}
