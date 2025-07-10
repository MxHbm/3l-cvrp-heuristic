#pragma once

#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/LoadingChecker.h"

#include "Model/Instance.h"
#include "Model/Solution.h"
#include "Algorithms/BCRoutingParams.h"

#include "Improvement/FullEnumerationSearch.h"
#include "Improvement/TwoOpt.h"
#include "Improvement/InterSwap.h"
#include "Improvement/IntraSwap.h"
#include "Improvement/K_RandomSwaps.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{
class LocalSearch
{
  public:
    static void RunLocalSearch(const Instance* const instance,
                                LoadingChecker* loadingChecker,
                                const InputParameters* inputParameters,
                                Solution& currentSolution){

        for(auto const& localSearchType : inputParameters->IteratedLocalSearch.localSearchTypes){

            switch(localSearchType){
                
                case LocalSearchTypes::TwoOpt:
                    TwoOpt::Run(instance, *inputParameters, loadingChecker, currentSolution);
                    break;

                case LocalSearchTypes::InterSwap:
                    InterSwap::Run(instance, *inputParameters, loadingChecker, currentSolution);
                    break;

                case LocalSearchTypes::IntraSwap:
                    IntraSwap::Run(instance, *inputParameters, loadingChecker, currentSolution);
                    break;

                case LocalSearchTypes::FullEnumeration:
                    FullEnumerationSearch::Run(instance, *inputParameters, loadingChecker, currentSolution);
                    break;

                case LocalSearchTypes::None:
                    break;

                default: 
                    break;

            }
        }
    };

    static void RunPerturbation(const Instance* const instance,
                                LoadingChecker* loadingChecker,
                                const InputParameters* inputParameters,
                                Solution& currentSolution,
                                std::mt19937& RNG)
    {
        for(const auto& perturb_type : inputParameters->IteratedLocalSearch.perturbationTypes){

            switch(perturb_type){

                case PerturbationTypes::K_RandomSwaps:
                    K_RandomSwaps::Run(instance, *inputParameters, loadingChecker, currentSolution, RNG);
                    break;
                
                case PerturbationTypes::None:
                    break;

                default: 
                    break;

            }
        }
    }
};

}
}