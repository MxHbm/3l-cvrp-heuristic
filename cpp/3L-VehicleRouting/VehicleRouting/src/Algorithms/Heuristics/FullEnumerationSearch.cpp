#include "Algorithms/Heuristics/FullEnumerationSearch.h"

#include <algorithm>

#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"

namespace VehicleRouting
{
namespace Algorithms
{
namespace Heuristics
{
namespace Improvement
{
void FullEnumerationSearch::Run(const Instance* const instance,
                                const InputParameters& inputParameters,
                                LoadingChecker* loadingChecker,
                                Collections::IdVector& route)
{
    auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

    auto routeCosts = Evaluator::CalculateRouteCosts(instance, route);

    auto tmpRoute = route;
    std::ranges::sort(tmpRoute);

    using move = std::pair<double, Collections::IdVector>;

    std::vector<move> moves;
    do
    {
        auto costs = Evaluator::CalculateRouteCosts(instance, tmpRoute);

        if (costs >= routeCosts)
        {
            continue;
        }

        moves.emplace_back(costs, tmpRoute);

    } while (std::next_permutation(std::begin(tmpRoute), std::end(tmpRoute)));

    std::ranges::sort(moves);

    const auto& container = instance->Vehicles.front().Containers.front();
    double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);

    for (auto& move: moves)
    {
        auto& sequence = move.second;

        if (loadingChecker->RouteIsInFeasSequences(sequence))
        {
            route = std::move(sequence);
            break;
        }

        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route))
        {
            loadingChecker->AddFeasibleSequenceFromOutside(sequence);
            route = std::move(sequence);
            break;
        }

        auto selectedItems = InterfaceConversions::SelectItems(sequence, instance->Nodes, false);
        auto status = loadingChecker->HeuristicCompleteCheck(container, set, sequence, selectedItems, maxRuntime);

        if (status == LoadingStatus::FeasOpt)
        {
            route = std::move(sequence);
            break;
        }

        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo)
        {
            route = std::move(sequence);
            break;
        }
    }
}

}
}
}
}