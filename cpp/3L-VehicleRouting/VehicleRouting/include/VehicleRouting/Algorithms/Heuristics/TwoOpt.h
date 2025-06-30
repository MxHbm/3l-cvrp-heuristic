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

using TwoOptMove = std::tuple<double, size_t, size_t>;

class TwoOpt
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    Solution& currentSolution);

  private:
    static std::vector<TwoOptMove> DetermineMoves(const Instance* instance, const Collections::IdVector& route);
    static double GetBestMove(const Instance* instance,
                              const InputParameters& inputParameters,
                              LoadingChecker* loadingChecker,
                              Collections::IdVector& route,
                              std::vector<TwoOptMove>& moves);

    static void ChangeRoutes(Collections::IdVector& route, size_t i, size_t k);
};

}
}
}
}