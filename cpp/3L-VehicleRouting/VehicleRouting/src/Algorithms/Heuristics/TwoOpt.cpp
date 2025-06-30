#include "Algorithms/Heuristics/TwoOpt.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "CommonBasics/Helper/ModelServices.h"



#include <algorithm>

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
using namespace ContainerLoading;

void TwoOpt::Run(const Instance* const instance,
                 const InputParameters& inputParameters,
                 LoadingChecker* loadingChecker,
                 Solution& currentSolution)
{
    for(auto& route : currentSolution.Routes){

        if (route.Sequence.size() < 3)
        {
            return;
        }

        if (loadingChecker->SequenceIsCheckedTwoOpt(route.Sequence))
        {
            return;
        }

        while (true)
        {
            auto moves = DetermineMoves(instance, route.Sequence);
            auto savings = GetBestMove(instance, inputParameters, loadingChecker, route.Sequence, moves);
            if (savings >= 0.0){
                break;
            }else{
                std::cout << "Savings - " << savings << std::endl;
                loadingChecker->AddSequenceCheckedTwoOpt(route.Sequence);
                currentSolution.Costs += savings;
                std::cout << "Costs afterwards - " << currentSolution.Costs << std::endl;
            }
        }
        return;
    }
}

std::vector<TwoOptMove> TwoOpt::DetermineMoves(const Instance* const instance,
                                               const Collections::IdVector& route)
{
    std::vector<TwoOptMove> moves = std::vector<TwoOptMove>();
    auto savings = 0.0; 

    for (size_t i = 0; i < route.size() - 1; ++i)
    {
        for (size_t k = i + 1; k < route.size(); ++k)
        {

            savings = Evaluator::CalculateTwoOptDelta(instance, route, i, k);

            if (savings < 0.0)
            {
                moves.emplace_back(savings, i, k);
            }
        }
    }

    return moves;
}

double TwoOpt::GetBestMove(const Instance* const instance,
                                            const InputParameters& inputParameters,
                                            LoadingChecker* loadingChecker,
                                            Collections::IdVector& route,
                                            std::vector<TwoOptMove>& moves)
    {

    auto default_return = 0.0; 

    if (moves.size() == 0)
    {
        return default_return;
    }

    std::ranges::sort(moves, [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
    });

    auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

    const auto& container = instance->Vehicles.front().Containers.front();
    double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);

    for (const auto& move: moves)
    {

        ChangeRoutes(route, std::get<1>(move), std::get<2>(move));

        if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
        {
            return std::get<0>(move);
        }

        // If lifo is disabled, feasibility of route is independent from actual sequence
        // -> move is always feasible if route is feasible
        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route))
        {
            return std::get<0>(move);
        }

        auto selectedItems = InterfaceConversions::SelectItems(route, instance->Nodes, false);
        auto status = loadingChecker->HeuristicCompleteCheck(container, set, route, selectedItems, maxRuntime);

        if (status == LoadingStatus::FeasOpt)
        {
            return std::get<0>(move);
        }
        
        //Change routes back if it was not feasible! 
        ChangeRoutes(route, std::get<1>(move), std::get<2>(move));
    }

    return default_return;
}


void TwoOpt::ChangeRoutes(Collections::IdVector& route, size_t i, size_t k)
{
    std::reverse(route.begin() + i, route.begin() + k + 1);
    
}

}
}
}
}