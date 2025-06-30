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
#include "Algorithms/Heuristics/K_RandomSwaps.h"

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

        std::cout << "Result after InterSwap" << std::endl;
        int id = 0;
        for(const auto& route : routes){

            int tot_vol = 0; 
            int tot_wei = 0;
            for(const auto& node : route.Sequence){
                tot_vol += instance->Nodes[node].TotalVolume;
                tot_wei += instance->Nodes[node].TotalWeight;
            }
            std::cout << "Route "<<id<<" - Total Weigth: " << tot_wei  << std::endl;
            ++id;
        }
    };

    static void RunPerturbation(const Instance* const instance,
                                    LoadingChecker* loadingChecker,
                                    const InputParameters* inputParameters,
                                    std::vector<Route>& routes,
                                    std::mt19937& RNG)
    {
        K_RandomSwaps::Run(instance, *inputParameters, loadingChecker, routes, RNG);

        std::cout << "Result after Perturbation" << std::endl;
        int id = 0;
        for(const auto& route : routes){

            int tot_vol = 0; 
            int tot_wei = 0;
            for(const auto& node : route.Sequence){
                tot_vol += instance->Nodes[node].TotalVolume;
                tot_wei += instance->Nodes[node].TotalWeight;
            }
            std::cout << "Route "<<id<<" - Total Weigth: " << tot_wei  << std::endl;
            ++id;
        }
    }
};

}
}
}
}