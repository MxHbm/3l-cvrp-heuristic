#include "Improvement/K_RandomSwaps.h"

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;

std::optional<PerturbationMove> K_RandomSwaps::DetermineMoves(const Instance* const instance,
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

                auto savings = Evaluator::CalculateInterSwapDelta(instance,
                                                                    route_i.Sequence,
                                                                    route_k.Sequence,
                                                                    node_i,
                                                                    node_k); 
                return std::make_tuple(savings, route_it_i, route_it_k, node_i, node_k, weight_item_delta, volume_item_delta);
        }
            
            ++attempts;
        }
        ++route_draws;
    }
    return std::nullopt;
}


void K_RandomSwaps::ChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move)
{

    auto node_i = std::get<3>(move);
    auto node_k = std::get<4>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    std::swap(route_i.Sequence[node_i], route_k.Sequence[node_k]);
    
}

void K_RandomSwaps::RevertChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move)
{

    auto node_i = std::get<3>(move);
    auto node_k = std::get<4>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    std::swap(route_i.Sequence[node_i], route_k.Sequence[node_k]);
    
}
}
}