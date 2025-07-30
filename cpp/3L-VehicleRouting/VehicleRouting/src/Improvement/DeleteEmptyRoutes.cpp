#include "Improvement/DeleteEmptyRoutes.h"

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;

void DeleteEmptyRoutes::Run(const Instance* instance,
            const InputParameters& inputParameters,
            LoadingChecker* loadingChecker,
            Classifier* classifier,
            Solution& currentSolution){

    // Remove empty routes
    auto& routes = currentSolution.Routes;

    routes.erase(std::remove_if(routes.begin(), routes.end(),
                                [](const Route& route) {
                                    return route.Sequence.empty();
                                }),
                 routes.end());

    // Reassign internal route IDs or reindex if necessary
    for (size_t i = 0; i < routes.size(); ++i)
    {
        routes[i].Id = static_cast<int>(i); // assuming Route has an Id field
    }
    // Update solution metadata
    currentSolution.NumberOfRoutes = routes.size();
};

}
}