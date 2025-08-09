#pragma once

#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "Model/Instance.h"
#include "Model/Solution.h"
#include "Algorithms/BCRoutingParams.h"
#include "Algorithms/LoadingInterfaceServices.h"

#include <optional>
#include <tuple>
#include <vector>

namespace VehicleRouting {
namespace Improvement {

class LocalSearchOperatorBase {
public:
    virtual ~LocalSearchOperatorBase() = default;

    virtual void Run(const Instance* instance,
                     const InputParameters& inputParameters,
                     BaseLoadingChecker* loadingChecker,
                     Solution& currentSolution) = 0;
};

}}  // namespace
