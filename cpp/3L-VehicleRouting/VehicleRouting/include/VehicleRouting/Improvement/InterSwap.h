#pragma once

#include "InterLocalSearchOperator.h"

namespace VehicleRouting
{

namespace Improvement
{

class InterSwap : public InterLocalSearchOperator
{
  private:
    std::vector<InterMove> DetermineMoves(const Instance* instance,
                                                     const std::vector<Route>& routes) override;

    void ChangeRoutes(std::vector<Route>& routes, const InterMove& move) override;
    void RevertChangeRoutes(std::vector<Route>& routes, const InterMove& move) override;

};
}
}