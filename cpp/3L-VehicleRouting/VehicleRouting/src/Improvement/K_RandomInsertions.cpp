#include "Improvement/K_RandomInsertions.h"

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;

std::optional<PerturbationMove> K_RandomInsertions::DetermineMoves(const Instance* const instance,
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

        //Insert node i from route i to route k at position! 
        const auto res_weight_route_k = container_weight_limit - route_k.TotalWeight;
        const auto res_volume_route_k = container_volume_limit - route_k.TotalVolume;

        std::uniform_int_distribution<size_t> route_i_dist(0, route_i.Sequence.size() - 1);
        std::uniform_int_distribution<size_t> route_k_dist(0, route_k.Sequence.size());

        int attempts = 0;

        //TODO check if this can be compared! 
        while(attempts <= 0.1 * route_i.Sequence.size() * route_k.Sequence.size()){

            const auto node_i = route_i_dist(RNG);
            const auto position_k = route_k_dist(RNG);
            const auto weight_item_delta = instance->Nodes[route_i.Sequence[node_i]].TotalWeight;
            const auto volume_item_delta = instance->Nodes[route_i.Sequence[node_i]].TotalVolume;

        
        if(res_weight_route_k - weight_item_delta >= 0)
            if(res_volume_route_k - volume_item_delta >= 0){

                auto savings = Evaluator::CalculateInsertionDelta(instance,
                                                                    route_i.Sequence,
                                                                    route_k.Sequence,
                                                                    node_i,
                                                                    position_k); 

                return std::make_tuple(savings, route_it_i, route_it_k, node_i, position_k, weight_item_delta, volume_item_delta);
        }
            
            ++attempts;
        }
        ++route_draws;
    }
    return std::nullopt;
}

void K_RandomInsertions::ChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move)
{

    auto node_i = std::get<3>(move);
    auto position_k = std::get<4>(move);

    auto& route_i = routes[std::get<1>(move)].Sequence;
    auto& route_k = routes[std::get<2>(move)].Sequence;

    // 1. grab an iterator to the element
    auto it = route_i.begin() + node_i;

    // 2. move‑construct the element in the destination …
    route_k.insert(route_k.begin() + position_k, std::move(*it));   // <‑‑ one move

    // 3. …and erase it from the source
    route_i.erase(it);
}

void K_RandomInsertions::RevertChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move)
{
    auto node_i     = std::get<3>(move);  // original position in route_i
    auto position_k = std::get<4>(move);  // inserted position in route_k

    auto& route_i = routes[std::get<1>(move)].Sequence;  // source in original move
    auto& route_k = routes[std::get<2>(move)].Sequence;  // destination in original move

    // 1. grab iterator to the element that was inserted into route_k
    auto it = route_k.begin() + position_k;

    // 2. move it back into route_i at the original position
    route_i.insert(route_i.begin() + node_i, std::move(*it));

    // 3. erase it from route_k
    route_k.erase(it);

};
}
}