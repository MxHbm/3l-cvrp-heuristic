#pragma once

#include "ContainerLoading/LoadingChecker.h"
#include "Model/Instance.h"
#include "Model/Solution.h"
#include "Algorithms/BCRoutingParams.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "ContainerLoading/Classifier.h"

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
                     LoadingChecker* loadingChecker,
                     Classifier* classifier,
                     Solution& currentSolution) = 0;
};

}}  // namespace
