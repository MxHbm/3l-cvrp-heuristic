
#include "Improvement/InterSwap.h"

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;


std::vector<InterMove> InterSwap::DetermineMoves(const Instance* const instance,
                                                     const std::vector<Route>& routes)
{

    std::vector<InterMove> moves{};
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


void InterSwap::ChangeRoutes(std::vector<Route>& routes, const InterMove& move)
{

    auto node_i = std::get<3>(move);
    auto node_k = std::get<4>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    std::swap(route_i.Sequence[node_i], route_k.Sequence[node_k]);
    
};

void InterSwap::RevertChangeRoutes(std::vector<Route>& routes, const InterMove& move)
{

    auto node_i = std::get<3>(move);
    auto node_k = std::get<4>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    std::swap(route_i.Sequence[node_i], route_k.Sequence[node_k]);
    
};

}
}
