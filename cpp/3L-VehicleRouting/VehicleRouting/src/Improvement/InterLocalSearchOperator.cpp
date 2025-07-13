#include "Improvement/InterLocalSearchOperator.h"
#include "CommonBasics/Helper/ModelServices.h"

namespace VehicleRouting
{
using namespace Model;
namespace Improvement
{
using namespace ContainerLoading;


void InterLocalSearchOperator::Run(const Instance* instance,
                    const InputParameters& inputParameters,
                    LoadingChecker* loadingChecker,
                    Classifier* classifier,
                    Solution& currentSolution){

  std::vector<Route>& routes = currentSolution.Routes;

  if (routes.size() < 2)
  {
      return;
  }

  while(true){

      auto moves = DetermineMoves(instance, routes);
      auto savings = GetBestMove(instance, inputParameters, loadingChecker,classifier, routes, moves);

      if(!savings){
          break;
      }else{
          currentSolution.Costs += *savings;
      }
  }

  return;

}

std::optional<double> InterLocalSearchOperator::GetBestMove(const Instance* instance,
                                const InputParameters& inputParameters,
                                LoadingChecker* loadingChecker,
                                ContainerLoading::Classifier* classifier,
                                std::vector<Route>& routes,
                                std::vector<InterMove>& moves){
  if (moves.size() == 0)
  {
      return std::nullopt;
  }

  std::ranges::sort(moves, [](const auto& a, const auto& b) {
      return std::get<0>(a) < std::get<0>(b);  // sort by savings ascending
  });
  
  //TODO - Create Bitset for all the two routes!
  //auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route);

  //Initiate variables before loop
  const double maxRuntime = inputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ExactLimit);
  const auto& container = instance->Vehicles.front().Containers.front();

  for (const auto& move: moves)
  {
      bool controlFlag = true;
      

      if (loadingChecker->Parameters.LoadingProblem.LoadingFlags == LoadingFlag::NoneSet)
      {
          UpdateRouteVolumeWeight(routes, move);
          return std::get<0>(move);
      }

      ChangeRoutes(routes, move);


      for(auto& route_index : {std::get<1>(move), std::get<2>(move)})
      {
        auto& route = routes[route_index];
        if(route.Sequence.empty()){
            continue;
        }

        auto selectedItems = Algorithms::InterfaceConversions::SelectItems(route.Sequence, instance->Nodes, false);
        // If lifo is disabled, feasibility of route is independent from actual sequence
        // -> move is always feasible if route is feasible
        if (!loadingChecker->Parameters.LoadingProblem.EnableLifo && loadingChecker->RouteIsInFeasSequences(route.Sequence))
        {
            continue;
        }

        if(inputParameters.ContainerLoading.classifierParams.UseClassifier){

            auto y = classifier->classify(selectedItems, route.Sequence, container, instance->Nodes.size(), instance->totalNoItems);
            //std::cout << "Output InterLocalSearch: " << y << std::endl;
            if (y <= inputParameters.ContainerLoading.classifierParams.AcceptanceThreshold)
            {
                controlFlag = false;
                break;
            }

        }else{

            auto set = loadingChecker->MakeBitset(instance->Nodes.size(), route.Sequence);
            auto status = loadingChecker->HeuristicCompleteCheck(container, set, route.Sequence, selectedItems, maxRuntime);

            if (status != LoadingStatus::FeasOpt)
            {
            controlFlag = false;
            break;
            }
        }

      }
      // Change routes back if not feasible!
      if (!controlFlag)
      {
          RevertChangeRoutes(routes, move);
          continue;
      }

      UpdateRouteVolumeWeight(routes, move);
      return std::get<0>(move);
  }

  return std::nullopt;

};


void InterLocalSearchOperator::UpdateRouteVolumeWeight(std::vector<Route>& routes, const InterMove& move){

    const auto item_delta = std::get<5>(move);
    const auto volume_delta = std::get<6>(move);

    auto& route_i = routes[std::get<1>(move)];
    auto& route_k = routes[std::get<2>(move)];

    route_i.TotalWeight -= item_delta;
    route_k.TotalWeight += item_delta;
    route_i.TotalVolume -= volume_delta;
    route_k.TotalVolume += volume_delta;
};

}
}
