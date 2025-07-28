
#include "Improvement/InterInsertion.h"

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;


std::vector<InterMove> InterInsertion::DetermineMoves(const Instance* const instance,
                                                 const std::vector<Route>& routes)
{

    std::vector<InterMove> moves{};
    const auto container_weight_limit = instance->Vehicles.front().Containers.front().WeightLimit;
    const auto container_volume_limit = instance->Vehicles.front().Containers.front().Volume;

    for (size_t route_it_i = 0; route_it_i < routes.size() - 1; ++route_it_i)
    {
        const auto& route_i = routes[route_it_i];

        for (size_t route_it_k = route_it_i + 1; route_it_k < routes.size(); ++route_it_k)
        {
            const auto& route_k = routes[route_it_k];
            const auto res_weight_route_k = container_weight_limit - route_k.TotalWeight;
            const auto res_volume_route_k = container_volume_limit - route_k.TotalVolume;

            for ( size_t node_i = 0; node_i < route_i.Sequence.size(); ++node_i)
            {
                const auto internId_node_i = route_i.Sequence[node_i];

                for (size_t position_k = 0; position_k <= route_k.Sequence.size(); ++position_k)
                {
                    const auto weight_item_delta = instance->Nodes[internId_node_i].TotalWeight;
                    const auto volume_item_delta = instance->Nodes[internId_node_i].TotalVolume;

                    if(res_weight_route_k - weight_item_delta >= 0)
                        if(res_volume_route_k - volume_item_delta >= 0){

                            auto savings = Evaluator::CalculateInsertionDelta(instance,
                                                    route_i.Sequence,
                                                    route_k.Sequence,
                                                    node_i,
                                                    position_k); 


                            if (savings < -1e-3)
                            {
                                moves.emplace_back(savings, route_it_i, route_it_k, node_i, position_k, weight_item_delta, volume_item_delta);
                            }
                    }
                }


            }
        }
    }

    return moves;
}


void InterInsertion::ChangeRoutes(std::vector<Route>& routes, const InterMove& move)
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

void InterInsertion::RevertChangeRoutes(std::vector<Route>& routes, const InterMove& move)
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

}

}
}
