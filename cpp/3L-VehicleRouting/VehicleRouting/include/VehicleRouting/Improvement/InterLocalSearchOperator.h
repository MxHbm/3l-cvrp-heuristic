#pragma once

#include "LocalSearchOperatorBase.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;


using InterMove = std::tuple<double, size_t, size_t, size_t, size_t, int, int>;

class InterLocalSearchOperator : public LocalSearchOperatorBase
{
  public:
    void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    ContainerLoading::Classifier*    classifier,
                    Solution& currentSolution) override;

  protected:
    std::optional<double> GetBestMove(const Instance* instance,
                                const InputParameters& inputParameters,
                                LoadingChecker* loadingChecker,
                                ContainerLoading::Classifier*    classifier,
                                std::vector<Route>& routes,
                                std::vector<InterMove>& moves);

    virtual std::vector<InterMove> DetermineMoves(const Instance* instance,
                                                  const std::vector<Route>& routes) = 0;

    virtual void ChangeRoutes(std::vector<Route>& routes, const InterMove& move) = 0;
    virtual void RevertChangeRoutes(std::vector<Route>& routes, const InterMove& move) = 0; 

    void UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterMove& move);

};
}
}