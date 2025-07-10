#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"
#include "Model/Solution.h"

#include "Algorithms/BCRoutingParams.h"

#include <random>
#include <optional>

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;


using InterSwapMove = std::tuple<double, size_t, size_t, size_t, size_t, int, int>;

class K_RandomSwaps
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    Solution& solution,
                    std::mt19937& RNG);

  private:
    static std::optional<InterSwapMove> DetermineMoves(const Instance* instance,
                                                        const std::vector<Route>& routes,
                                                        std::mt19937& RNG);

    static void ChangeRoutes(std::vector<Route>& routes, const InterSwapMove& move);

    static void UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterSwapMove& move);

};
}
}