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

    void ChangeRoute(Collections::IdVector& route, const size_t i, const size_t k) override;

    void RevertRoute(Collections::IdVector& route, const size_t k, const size_t i) override;
};

}
}