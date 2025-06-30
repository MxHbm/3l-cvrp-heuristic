#pragma once

#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"
#include "Model/Solution.h"

#include "Algorithms/BCRoutingParams.h"

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


using InterSwapMove = std::tuple<double, size_t, size_t, size_t, size_t, int, int>;

class InterSwap
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    Solution& currentSolution);

  private:
    static std::vector<InterSwapMove> DetermineMoves(const Instance* instance,
                                                     const std::vector<Route>& routes);
    static double GetBestMove(const Instance* instance,
                      const InputParameters& inputParameters,
                      LoadingChecker* loadingChecker,
                      std::vector<Route>& routes,
                      std::vector<InterSwapMove>& moves);

    static void ChangeRoutes(std::vector<Route>& routes, const InterSwapMove& move);

    static void UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterSwapMove& move);

};
}
}
}
}