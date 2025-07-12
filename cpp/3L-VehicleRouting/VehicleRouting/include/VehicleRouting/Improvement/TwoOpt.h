#pragma once

#include "IntraLocalSearchOperator.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{

using namespace ContainerLoading;

class TwoOpt : public IntraLocalSearchOperator
{
  private:
    std::vector<IntraMove> DetermineMoves(const Instance* instance,
                                          const Collections::IdVector& route) override;

    void ChangeRoute(Collections::IdVector& route, size_t i, size_t k) override;
};

}
}