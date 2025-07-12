#pragma once

#include "PerturbationOperatorBase.h"

namespace VehicleRouting
{

namespace Improvement
{

class K_RandomInsertions : public PerturbationOperatorBase
{
  private:
     std::optional<PerturbationMove> DetermineMoves(const Instance* instance,
                                                    const std::vector<Route>& routes,
                                                    std::mt19937& rng) override;

    void ChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move) override;
    void RevertChangeRoutes(std::vector<Route>& routes, const PerturbationMove& move) override;

};
}
}