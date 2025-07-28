#pragma once

#include "LocalSearchOperatorBase.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
using namespace ContainerLoading;

class DeleteEmptyRoutes : public LocalSearchOperatorBase
{
    void Run(const Instance* instance,
            const InputParameters& inputParameters,
            LoadingChecker* loadingChecker,
            Classifier* classifier,
            Solution& currentSolution) override;

};

}
}