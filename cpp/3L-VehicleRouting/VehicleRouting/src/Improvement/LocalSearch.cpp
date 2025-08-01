

#include "Improvement/LocalSearch.h"

namespace VehicleRouting
{
using namespace Model;

namespace Improvement
{

// Build operator lists once, from whatever vectors your config provides
LocalSearch::LocalSearch(const InputParameters& params,
                        const Instance* inst)
                        : mInstance(inst),
                        mInputParameters(params)
{   
    for (auto t : params.IteratedLocalSearch.localSearchTypes)
        if (auto op = this->CreateLocalSearchOperator(t))
            lsOperators.emplace_back(std::move(op));

    for (auto t : params.IteratedLocalSearch.perturbationTypes)
        if (auto op = this->CreatePerturbationOperator(t))
            pertOperators.emplace_back(std::move(op));
}


// Run all local‑search moves in order
void LocalSearch::RunLocalSearch(Model::Solution& sol,
                                ContainerLoading::LoadingChecker* checker)
{
    for (auto& op : lsOperators){
        op->Run(mInstance, mInputParameters, checker, sol);
    }
};

// Run all perturbations in order
void LocalSearch::RunPerturbation(Model::Solution&                  sol,
                                  ContainerLoading::LoadingChecker* checker,
                                  std::mt19937&                     rng)
{
    for (auto& op : pertOperators){
        //TODO handles nullptr case! 
        if(op != nullptr){
            op->Run(mInstance, mInputParameters, checker, sol, rng);
        }
        break;
    }
};

// Run all perturbations in order
void LocalSearch::RunBigPerturbation(Model::Solution&                  sol,
                                  ContainerLoading::LoadingChecker* checker,
                                  std::mt19937&                     rng)
{
    for (auto& op : pertOperators)
        //TODO handles nullptr case! 
        if(op != nullptr){
            op->Run(mInstance, mInputParameters, checker, sol, rng);
        }
};

std::unique_ptr<LocalSearchOperatorBase> LocalSearch::CreateLocalSearchOperator(const LocalSearchTypes& t)
{
    switch (t)
    {
        case LocalSearchTypes::TwoOpt:          
            return std::make_unique<TwoOpt>();
        case LocalSearchTypes::IntraSwap:       
            return std::make_unique<IntraSwap>();
        case LocalSearchTypes::IntraInsertion:       
            return std::make_unique<IntraInsertion>();
        case LocalSearchTypes::InterSwap:       
            return std::make_unique<InterSwap>();
        case LocalSearchTypes::InterInsertion:       
            return std::make_unique<InterInsertion>();
        case LocalSearchTypes::DeleteEmptyRoutes:       
            return std::make_unique<DeleteEmptyRoutes>();
        //case LocalSearchTypes::FullEnumeration: 
        //    return std::make_unique<FullEnumerationSearch>();
        default:                                
            return nullptr;                       
    }
}

std::unique_ptr<PerturbationOperatorBase> LocalSearch::CreatePerturbationOperator(const PerturbationTypes& t)
{
        switch (t)
        {
            case PerturbationTypes::K_RandomSwaps:  
                return std::make_unique<K_RandomSwaps>();
            case PerturbationTypes::K_RandomInsertions:
                return std::make_unique<K_RandomInsertions>();
            default:                               
                return nullptr;
        }
};

}} // namespace VehicleRouting::Improvement


/*

private:

    std::unique_ptr<LocalSearchOperatorBase> CreateLocalSearchOperator(LocalSearchTypes t)
    {
        switch (t)
        {
            case LocalSearchTypes::TwoOpt:          
                return std::make_unique<TwoOptOperator>();
            case LocalSearchTypes::IntraSwap:       
                return std::make_unique<IntraSwapOperator>();
            case LocalSearchTypes::InterSwap:       
                return std::make_unique<InterSwap>();
            case LocalSearchTypes::FullEnumeration: 
                return std::make_unique<FullEnumerationOperator>();
            default:                                
                return nullptr;
        }
    }

*/