#pragma once

#include "Improvement/FullEnumerationSearch.h"
#include "Improvement/TwoOpt.h"
#include "Improvement/InterSwap.h"
#include "Improvement/IntraSwap.h"
#include "Improvement/InterInsertion.h"
#include "Improvement/IntraInsertion.h"
#include "Improvement/K_RandomSwaps.h"
#include "Improvement/K_RandomInsertions.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{

class LocalSearch
{
public:
    // Build operator lists once, from whatever vectors your config provides
    LocalSearch(const InputParameters& params,
                const Instance* inst);

    // Run all local‑search moves in order
    void RunLocalSearch(Solution& sol,
                        ContainerLoading::LoadingChecker* checker,
                        ContainerLoading::Classifier* classifier);

    // Run all perturbations in order
    void RunPerturbation(Solution& sol,
                        ContainerLoading::LoadingChecker* checker,
                        ContainerLoading::Classifier* classifier,
                         std::mt19937& rng);

        // Run all perturbations in order
    void RunBigPerturbation(Solution& sol,
                        ContainerLoading::LoadingChecker* checker,
                        ContainerLoading::Classifier* classifier,
                         std::mt19937& rng);

private:
    std::vector<std::unique_ptr<LocalSearchOperatorBase>>  lsOperators;
    std::vector<std::unique_ptr<PerturbationOperatorBase>> pertOperators;
    const Instance* mInstance = nullptr;
    const InputParameters mInputParameters;

    std::unique_ptr<LocalSearchOperatorBase> CreateLocalSearchOperator(const LocalSearchTypes& t);
    std::unique_ptr<PerturbationOperatorBase> CreatePerturbationOperator(const PerturbationTypes& t);
};

}} // namespace VehicleRouting::Improvement


