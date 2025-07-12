#pragma once

#include "LocalSearchOperatorBase.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

using IntraMove = std::tuple<double, size_t, size_t>;

class IntraLocalSearchOperator : public LocalSearchOperatorBase
{
  public:
    void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    Solution& currentSolution) override;

  protected:
    std::optional<double> GetBestMove(const Instance* instance,
                                    const InputParameters& inputParameters,
                                    LoadingChecker* loadingChecker,
                                    Collections::IdVector& route,
                                    std::vector<IntraMove>& moves);

    virtual std::vector<IntraMove> DetermineMoves(const Instance* instance,
                                                  const Collections::IdVector& route) = 0;

    virtual void ChangeRoute(Collections::IdVector& route, const size_t node_i, const size_t node_k) = 0;
};
}
}