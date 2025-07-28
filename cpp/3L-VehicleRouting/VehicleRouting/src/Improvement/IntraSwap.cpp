#include "Improvement/IntraSwap.h"

#include <algorithm>

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;

std::vector<IntraMove> IntraSwap::DetermineMoves(const Instance* const instance,
                                           const Collections::IdVector& route)
{

    std::vector<IntraMove> moves{};
    auto savings = 0.0; 

    for (size_t i = 0; i < route.size() - 1; ++i)
    {
        for (size_t k = i + 1; k < route.size(); ++k)
        {

            savings = Evaluator::CalculateIntraSwapDelta(instance, route, i, k);

            if (savings < -1e-3)
            {
                moves.emplace_back(savings, i, k);
            }
        }
    }

    return moves;
}

void IntraSwap::ChangeRoute(Collections::IdVector& route, const size_t node_i, const size_t node_k)
{
    std::swap(route[node_i], route[node_k]);
};


void IntraSwap::RevertRoute(Collections::IdVector& route, const size_t node_i, const size_t node_k)
{
    std::swap(route[node_i], route[node_k]);
};

}
}