#include "Improvement/FullEnumerationSearch.h"

#include <algorithm>

#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"

namespace VehicleRouting
{
namespace Improvement
{
void FullEnumerationSearch::Run(const Instance* const instance,
                                const InputParameters& inputParameters,
                                LoadingChecker* loadingChecker,
                                Solution& currentSolution)
{
    for(auto& route : currentSolution.Routes){

        auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);

        auto routeCosts = Evaluator::CalculateRouteCosts(instance, route.Sequence);

        auto tmpRoute = route.Sequence;
        std::ranges::sort(tmpRoute);

        using move = std::pair<double, Collections::IdVector>;

        std::vector<move> moves;
        do
        {
            auto costs = Evaluator::CalculateRouteCosts(instance, tmpRoute);
            auto delta_routeCosts = costs - routeCosts;

            if (delta_routeCosts > 0)
            {
                continue;
            }

            moves.emplace_back(delta_routeCosts, tmpRoute);

        } while (std::next_permutation(std::begin(tmpRoute), std::end(tmpRoute)));

        std::ranges::sort(moves);

        const auto& container = instance->Vehicles.front().Containers.front();
        double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);

        for (auto& move: moves)
        {
            auto& sequence = move.second;

            if (loadingChecker->RouteIsInFeasSequences(sequence))
            {
                route.Sequence = std::move(sequence);
                currentSolution.Costs += move.first;
                break;
            }

            if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route.Sequence))
            {
                loadingChecker->AddFeasibleSequenceFromOutside(sequence);
                route.Sequence = std::move(sequence);
                currentSolution.Costs += move.first;
                break;
            }

            auto selectedItems = InterfaceConversions::SelectItems(sequence, instance->Nodes, false);
            auto status = loadingChecker->CompleteCheck(container, set, sequence, selectedItems, maxRuntime);

            if (status == LoadingStatus::FeasOpt)
            {
                route.Sequence = std::move(sequence);
                currentSolution.Costs += move.first;
                break;
            }

            if (!loadingChecker->Parameters.LoadingProblem.EnableLifo)
            {
                route.Sequence = std::move(sequence);
                currentSolution.Costs += move.first;
                break;
            }
        }
    }
}

}
}