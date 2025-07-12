#pragma once

#include "IntraLocalSearchOperator.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

class IntraSwap : public IntraLocalSearchOperator
{
  private:
    std::vector<IntraMove> DetermineMoves(const Instance* instance, const Collections::IdVector& route) override;

    void ChangeRoute(Collections::IdVector& route, const size_t node_i, const size_t node_k) override;
};


}
}