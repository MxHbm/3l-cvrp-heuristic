#pragma once

#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "Model/Instance.h"
#include "Model/Solution.h"
#include "Algorithms/BCRoutingParams.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

class FullEnumerationSearch
{
  public:
    static void Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    BaseLoadingChecker* loadingChecker,
                    Solution& currentSolution);
};

}
}