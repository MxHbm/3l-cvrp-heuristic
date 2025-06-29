#pragma once

#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"
#include "Model/Solution.h"

#include "Algorithms/BCRoutingParams.h"

#include "Algorithms/Heuristics/FullEnumerationSearch.h"
#include "Algorithms/Heuristics/TwoOpt.h"
#include "Algorithms/Heuristics/InterSwap.h"
#include "Algorithms/Heuristics/IntraSwap.h"

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
class LocalSearch
{
  public:
    static void RunIntraImprovement(const Instance* const instance,
                                    LoadingChecker* loadingChecker,
                                    const InputParameters* inputParameters,
                                    Collections::IdVector& sequence)
    {
        if (!inputParameters->IteratedLocalSearch.ActivateIntraRouteImprovement)
        {
            return;
        }

        if (sequence.size() < 3)
        {
            return;
        }

        if (sequence.size() < inputParameters->IteratedLocalSearch.IntraRouteFullEnumThreshold)
        {
            FullEnumerationSearch::Run(instance, *inputParameters, loadingChecker, sequence);
        }
        else
        {
            TwoOpt::Run(instance, *inputParameters, loadingChecker, sequence);
        }
    };

    static void RunInterImprovement(const Instance* const instance,
                                    LoadingChecker* loadingChecker,
                                    const InputParameters* inputParameters,
                                    std::vector<Route>& routes)
    {
        InterSwap::Run(instance, *inputParameters, loadingChecker, routes);
    };
};

}
}
}
}