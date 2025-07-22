#include "Improvement/TwoOpt.h"
#include <algorithm>

namespace VehicleRouting
{
namespace Improvement
{
using namespace ContainerLoading;

std::vector<IntraMove> TwoOpt::DetermineMoves(const Instance* const instance,
                                               const Collections::IdVector& route)
{
    std::vector<IntraMove> moves = std::vector<IntraMove>();
    auto savings = 0.0; 

    for (size_t i = 0; i < route.size() - 1; ++i)
    {
        for (size_t k = i + 1; k < route.size(); ++k)
        {

            savings = Evaluator::CalculateTwoOptDelta(instance, route, i, k);

            if (savings < -1e-3)
            {
                moves.emplace_back(savings, i, k);
            }
        }
    }

    return moves;
}



void TwoOpt::ChangeRoute(Collections::IdVector& route, const size_t i, const size_t k)
{
    std::reverse(route.begin() + i, route.begin() + k + 1);
    
}


void TwoOpt::RevertRoute(Collections::IdVector& route, const size_t i, const size_t k){

    std::reverse(route.begin() + i, route.begin() + k + 1);

}

}
}


//TODO: Old Run method --> discuss with Florian linß if important to keep
/*
void TwoOpt::Run(const Instance* const instance,
                 const InputParameters& inputParameters,
                 LoadingChecker* loadingChecker,
                 Solution& currentSolution)
{
    for(auto& route : currentSolution.Routes){

        if (route.Sequence.size() < 3)
        {
            continue;
        }

        if (loadingChecker->SequenceIsCheckedTwoOpt(route.Sequence))
        {
            continue;
        }

        while (true)
        {
            auto moves = DetermineMoves(instance, route.Sequence);
            auto savings = GetBestMove(instance, inputParameters, loadingChecker, route.Sequence, moves);
            if (!savings){
                break;
            }else{
                loadingChecker->AddSequenceCheckedTwoOpt(route.Sequence);
                currentSolution.Costs += *savings;
            }
        }
    }
}
*/