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

namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
using namespace ContainerLoading;


using K_InterSwapMove = std::tuple<size_t, size_t, size_t, size_t, int, int>;

class K_RandomSwaps
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    std::vector<Route>& routes,
                    std::mt19937& RNG);

  private:
    static std::optional<K_InterSwapMove> DetermineMoves(const Instance* instance,
                                                        const std::vector<Route>& routes,
                                                        std::mt19937& RNG);

    static void ChangeRoutes(std::vector<Route>& routes, const K_InterSwapMove& move);

    static void UpdateRouteVolumeWeight(std::vector<Route>& routes, const K_InterSwapMove& move);

};
}
}
}
}