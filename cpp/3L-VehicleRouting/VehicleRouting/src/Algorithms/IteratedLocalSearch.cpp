#include "Algorithms/IteratedLocalSearch.h"

#include "Algorithms/Evaluation.h"
#include "CommonBasics/Helper/ModelServices.h"
#include "ContainerLoading/Algorithms/CPSolverParameters.h"
#include "ContainerLoading/Algorithms/LoadingStatus.h"
#include "ContainerLoading/Helper/HelperIO.h"

#include "Algorithms/Heuristics/Constructive.h"
#include "Algorithms/Heuristics/LocalSearch.h"
//#include "Algorithms/Heuristics/SPHeuristic.h"
#include "Helper/HelperIO.h"
#include "Helper/Serialization.h"
#include "Model/Instance.h"
#include "Model/Solution.h"

#include "Algorithms/LoadingInterfaceServices.h"

#include <cstdint>
#include <memory>

namespace VehicleRouting
{
using namespace Model;

namespace Algorithms
{
namespace CLP = ContainerLoading;
using namespace ContainerLoading;
using namespace ContainerLoading::Algorithms;
using namespace Heuristics::Improvement;
using namespace Helper;

void IteratedLocalSearch::Initialize()
{
    mLogFile << "ProblemVariant: " << (int)mInputParameters.ContainerLoading.LoadingProblem.Variant << "\n";

    std::vector<Container> containers;
    containers.reserve(mInstance->Vehicles.size());
    for (const auto& vehicle: mInstance->Vehicles)
    {
        containers.emplace_back(vehicle.Containers[0]);
    }

    std::vector<Group> customerNodes;
    customerNodes.reserve(mInstance->GetCustomers().size());
    for (const auto& node: mInstance->GetCustomers())
    {
        customerNodes.emplace_back(node.InternId,
                                   node.ExternId,
                                   node.PositionX,
                                   node.PositionY,
                                   node.TotalWeight,
                                   node.TotalVolume,
                                   node.TotalArea,
                                   node.Items);
    }

    mLoadingChecker = std::make_unique<LoadingChecker>(mInputParameters.ContainerLoading);
    mLoadingChecker->SetBinPackingModel(mEnv, containers, customerNodes, mOutputPath);
    
    //Initialize Classifier: 
    mClassifier = std::make_unique<Classifier::MLModelsContainer>("C:/Users/mahu123a/Documents/train_classifier_model/mlp_cp_status_model_traced.pt");

    for (const auto& customer: mInstance->GetCustomers())
    {
        Collections::IdVector route = {customer.InternId};

        auto items = InterfaceConversions::SelectItems(route, mInstance->Nodes, false);

        auto heurStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, containers[0], route, items);

        if (heurStatus == LoadingStatus::FeasOpt)
        {
            continue;
        }

        auto exactStatus =
            mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                         containers[0],
                                                         mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route),
                                                         route,
                                                         items,
                                                         mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact));

        if (exactStatus != LoadingStatus::FeasOpt)
        {
            mLogFile << "Single customer route with " << customer.InternId << "is infeasible.\n";

            auto relStatus = mLoadingChecker->ConstraintProgrammingSolver(
                PackingType::NoSupport,
                containers[0],
                mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route),
                route,
                items,
                mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact));

            if (relStatus != LoadingStatus::FeasOpt)
            {
                throw std::runtime_error("Single customer route is infeasible!");
            }
        }
    }

    mRNG.seed(1008);
}


void IteratedLocalSearch::AdaptWeightsVolumesToLoadingProblem()
{
    mLogFile << "### START PREPROCESSING ###"
             << "\n";
    switch (mInputParameters.ContainerLoading.LoadingProblem.Variant)
    {
        case CLP::LoadingProblemParams::VariantType::Weight:
        {
            for (auto& node: mInstance->Nodes)
            {
                node.TotalVolume = 0.0;
                for (auto& item: node.Items)
                {
                    item.Volume = 0.0;
                }
            }

            for (auto& vehicle: mInstance->Vehicles)
            {
                for (auto& container: vehicle.Containers)
                {
                    container.Volume = 0.0;
                }
            }

            break;
        }
        case CLP::LoadingProblemParams::VariantType::Volume:
        {
            for (auto& node: mInstance->Nodes)
            {
                node.TotalWeight = 0.0;
                for (auto& item: node.Items)
                {
                    item.Weight = 0.0;
                }
            }

            for (auto& vehicle: mInstance->Vehicles)
            {
                for (auto& container: vehicle.Containers)
                {
                    container.WeightLimit = 0.0;
                }
            }

            break;
        }
        default:
            break;
    };
}

void IteratedLocalSearch::StartSolutionProcedure()
{
    mLogFile << "## Start start solution procedure ##\n";

    using enum IteratedLocalSearchParams::StartSolutionType;

    std::chrono::time_point<std::chrono::system_clock> start{std::chrono::system_clock::now()};

    switch (mInputParameters.IteratedLocalSearch.StartSolution)
    {
        case None:
            return;
        case ModifiedSavings:
            GenerateStartSolutionSavings();
            break;
        // TODO Decide what is here happening
        //case SPHeuristic:
        //    GenerateStartSolutionSPHeuristic();
        //    break;
        default:
            throw std::runtime_error("Start solution type not implemented.");
    }

    mTimer.StartSolution = std::chrono::system_clock::now() - start;
    mCurrentSolution.NumberOfRoutes = mCurrentSolution.Routes.size();
    mCurrentSolution.DetermineCosts(mInstance);
    //TODO Change in Repar Modified Saavings, that Totalvoluem and Totalweight is updated! 
    mCurrentSolution.DeterminWeightsVolumes(mInstance);

    // TODO change, here just for intializing
    mBestSolution = mCurrentSolution;
    mSolutionTracker.UpdateBothSolutions(-mTimer.StartSolution.count(), mBestSolution.Costs);

    OutputSolution outputSolution(mCurrentSolution, mInstance);

    // Save values of start solution
    mLogFile << "Start solution with " << outputSolution.NumberOfRoutes << " Vehicles and total costs "
             << outputSolution.Costs << " in " << mTimer.StartSolution.count() << " s.\n";

    std::string solutionString = "StartSolution-" + mInstance->Name;
    Serializer::WriteToJson(outputSolution, mOutputPath, solutionString);

    mLogFile << "### END PREPROCESSING ###\n";
}

void IteratedLocalSearch::GenerateStartSolutionSavings()
{
    mCurrentSolution.Routes =
        Heuristics::Constructive::ModifiedSavings(mInstance, &mInputParameters, mLoadingChecker.get(), &mRNG).Run();

    //TODO what should i do with these values? 
    
    int id = 0;
    //Label routes new
    for (auto& route : mCurrentSolution.Routes)
    {
        route.Id = id;
        ++id;
    }
        
    /*
    for (const auto& route: mLoadingChecker->GetFeasibleRoutes())
    {
        if (route.size() < 2)
        {
            continue;
        }

        LocalSearch::RunIntraImprovement(mInstance, mLoadingChecker.get(), &mInputParameters, route);
    }
    */
}

/*
void IteratedLocalSearch::GenerateStartSolutionSPHeuristic()
{
    auto spHeuristic = Heuristics::SetBased::SPHeuristic(mInstance, mLoadingChecker.get(), &mInputParameters, mEnv);

    auto sequences = spHeuristic.Run(std::numeric_limits<double>::max());

    //Transform Collections::SequenceVector to std::vector<Route>
    std::vector<Route> solution;
    int id = 0;
    for (auto& sequence: *sequences)
    {
        solution.emplace_back(id++, sequence);
    }

    mCurrentSolution.Routes = std::move(solution);
}
*/

void IteratedLocalSearch::InfeasibleArcProcedure()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    DetermineInfeasiblePaths();
    mLogFile << "Deleted arcs: " << std::to_string(mInfeasibleArcs.size()) << "\n";
    mLogFile << "Infeasible tail paths: " << std::to_string(mInfeasibleTailPaths.size()) << "\n";
    mLogFile << "Infeasible 2-node combinations: " << std::to_string(mLoadingChecker->GetSizeInfeasibleCombinations())
             << "\n";

    DetermineExtendedInfeasiblePath();

    mLogFile << "Deleted arcs: " << std::to_string(mInfeasibleArcs.size()) << "\n";
    mLogFile << "Infeasible tail paths: " << std::to_string(mInfeasibleTailPaths.size()) << "\n";
    mLogFile << "Infeasible 2-node combinations: " << std::to_string(mLoadingChecker->GetSizeInfeasibleCombinations())
             << "\n";

    mTimer.InfeasibleArcs = std::chrono::system_clock::now() - start;
    // DetermineInfeasibleCustomerCombinations();
    // mLogFile << "Infeasible customer combinations size 3: " << std::to_string(mInfeasibleCombinations.size()) <<
    // "\n";
}

void IteratedLocalSearch::DetermineInfeasiblePaths()
{
    mLogFile << "## Start infeasible path procedure ## "
             << "\n";

    auto& container = mInstance->Vehicles.front().Containers.front();
    const auto& nodes = mInstance->Nodes;
    for (size_t iNode = 1; iNode < nodes.size() - 1; ++iNode)
    {
        // mLogFile << "Node: " << std::to_string(iNode) << "\n";
        for (size_t jNode = iNode + 1; jNode < nodes.size(); ++jNode)
        {
            if (nodes[iNode].TotalWeight + nodes[jNode].TotalWeight > container.WeightLimit
                || nodes[iNode].TotalVolume + nodes[jNode].TotalVolume > container.Volume)
            {
                boost::dynamic_bitset<> nodesInSet(nodes.size());
                nodesInSet.set(iNode).set(jNode);
                mLoadingChecker->AddInfeasibleCombination(nodesInSet);
                mInfeasibleArcs.emplace_back(0, iNode, jNode);
                mInfeasibleArcs.emplace_back(0, jNode, iNode);
                continue;
            }

            if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
            {
                continue;
            }

            Collections::IdVector selectedNodes = {iNode, jNode};
            auto selectedItems = InterfaceConversions::SelectItems(selectedNodes, mInstance->Nodes, false);
            bool forwardRelaxedInfeasible = !CheckPath(selectedNodes, container, selectedItems);

            std::swap(selectedNodes[0], selectedNodes[1]);
            selectedItems = InterfaceConversions::SelectItems(selectedNodes, mInstance->Nodes, false);
            bool backwardRelaxedInfeasible = !CheckPath(selectedNodes, container, selectedItems);

            if (forwardRelaxedInfeasible && backwardRelaxedInfeasible)
            {
                boost::dynamic_bitset<> nodesInSet(nodes.size());
                nodesInSet.set(iNode).set(jNode);
                mLoadingChecker->AddInfeasibleCombination(nodesInSet);
            }
        }
    }
}

bool IteratedLocalSearch::CheckPath(const Collections::IdVector& path, Container& container, std::vector<Cuboid>& items)
{   
    /*
    if (mInputParameters.IteratedLocalSearch.ActivateHeuristic)
    {
        auto heuristicStatus = mLoadingChecker->PackingHeuristic(PackingType::Complete, container, path, items);
        if (heuristicStatus == LoadingStatus::FeasOpt)
        {
            return true;
        }
    }
    */
    auto statusSupportRelaxation =
        mLoadingChecker->ConstraintProgrammingSolver(PackingType::NoSupport,
                                                     container,
                                                     mLoadingChecker->MakeBitset(mInstance->Nodes.size(), path),
                                                     path,
                                                     items,
                                                     mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact));

    if (statusSupportRelaxation == LoadingStatus::Infeasible)
    {
        mInfeasibleArcs.emplace_back(0, path.front(), path.back());
        return false;
    }

    auto statusComplete =
        mLoadingChecker->ConstraintProgrammingSolver(PackingType::Complete,
                                                     container,
                                                     mLoadingChecker->MakeBitset(mInstance->Nodes.size(), path),
                                                     path,
                                                     items,
                                                     mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact));

    if (statusComplete == LoadingStatus::Infeasible)
    {
        mInfeasibleTailPaths.emplace_back(0, path.front(), path.back());

        Collections::IdVector sequence = {path.front(), path.back()};
        mLoadingChecker->AddTailTournamentConstraint(sequence);
    }

    return true;
}

void IteratedLocalSearch::DetermineExtendedInfeasiblePath()
{
    if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
    {
        return;
    }

    mLogFile << "## Start extended infeasible path procedure ##\n";

    auto& container = mInstance->Vehicles.front().Containers.front();
    const auto& nodes = mInstance->Nodes;

    std::vector<Arc> tailPathToDelete;

    for (auto& arc: mInfeasibleTailPaths)
    {
        const auto& nodeI = nodes[arc.Tail];
        const auto& nodeJ = nodes[arc.Head];

        auto weight = nodeI.TotalWeight + nodeJ.TotalWeight;
        auto volume = nodeI.TotalVolume + nodeJ.TotalVolume;

        auto extraNodeFeasible = false;
        for (const auto& nodeK: mInstance->GetCustomers())
        {
            if (nodeK.InternId == nodeI.InternId || nodeK.InternId == nodeJ.InternId)
            {
                continue;
            }

            if (weight + nodeK.TotalWeight > container.WeightLimit || volume + nodeK.TotalVolume > container.Volume)
            {
                continue;
            }

            Collections::IdVector path = {nodeI.InternId, nodeJ.InternId, nodeK.InternId};

            auto selectedItems = InterfaceConversions::SelectItems(path, mInstance->Nodes, false);

            auto heuristicStatus = LoadingStatus::Infeasible;

            /*
            if (mInputParameters.IteratedLocalSearch.ActivateHeuristic)
            {
                heuristicStatus =
                    mLoadingChecker->PackingHeuristic(PackingType::Complete, container, path, selectedItems);
            }
            */

            if (heuristicStatus == LoadingStatus::Infeasible)
            {
                auto statusSupportRelaxation = mLoadingChecker->ConstraintProgrammingSolver(
                    PackingType::NoSupport,
                    container,
                    mLoadingChecker->MakeBitset(mInstance->Nodes.size(), path),
                    path,
                    selectedItems,
                    mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact));

                if (statusSupportRelaxation == LoadingStatus::Infeasible)
                {
                    continue;
                }
            }

            extraNodeFeasible = true;
            break;
        }

        if (extraNodeFeasible)
        {
            continue;
        }

        mInfeasibleArcs.emplace_back(0.0, nodeI.InternId, nodeJ.InternId);
        tailPathToDelete.emplace_back(arc);
    }

    for (const auto& path: tailPathToDelete)
    {
        std::erase_if(mInfeasibleTailPaths,
                      [path](Arc& arc) { return path.Head == arc.Head && path.Tail == arc.Tail; });
    }
}

void IteratedLocalSearch::DetermineInfeasibleCustomerCombinations()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    Container& container = mInstance->Vehicles.front().Containers.front();
    auto& nodes = mInstance->Nodes;

    for (size_t iNode = 1; iNode < nodes.size() - 2; ++iNode)
    {
        for (size_t jNode = iNode + 1; jNode < nodes.size() - 1; ++jNode)
        {
            for (size_t kNode = jNode + 1; kNode < nodes.size(); ++kNode)
            {
                double totalWeight = nodes[iNode].TotalWeight + nodes[jNode].TotalWeight + nodes[kNode].TotalWeight;
                double totalVolume = nodes[iNode].TotalVolume + nodes[jNode].TotalVolume + nodes[kNode].TotalVolume;

                if (totalWeight > container.WeightLimit || totalVolume > container.Volume)
                {
                    boost::dynamic_bitset<> nodesInSet(nodes.size());
                    nodesInSet.set(iNode);
                    nodesInSet.set(jNode);
                    nodesInSet.set(kNode);
                    mLoadingChecker->AddInfeasibleCombination(nodesInSet);
                    mInfeasibleCombinations.insert({iNode, jNode, kNode});
                    continue;
                }

                Collections::IdVector path = {iNode, jNode, kNode};

                auto selectedItems = InterfaceConversions::SelectItems(path, mInstance->Nodes, false);

                // Relaxed stability: supportAreaRequirement = 0.0.
                auto heuristicStatus =
                    mLoadingChecker->PackingHeuristic(PackingType::Complete, container, path, selectedItems);

                if (heuristicStatus == LoadingStatus::Infeasible)
                {
                    boost::dynamic_bitset<> nodesInSet(nodes.size());
                    nodesInSet.set(iNode);
                    nodesInSet.set(jNode);
                    nodesInSet.set(kNode);

                    double maxRuntime = mInputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::Exact);
                    auto status = mLoadingChecker->ConstraintProgrammingSolver(
                        PackingType::NoSupportNoSequence,
                        container,
                        nodesInSet,
                        path,
                        selectedItems,
                        mInputParameters.IsExact(IteratedLocalSearchParams::CallType::Exact),
                        maxRuntime);

                    if (status == LoadingStatus::Infeasible)
                    {
                        mLoadingChecker->AddInfeasibleCombination(nodesInSet);
                        mInfeasibleCombinations.insert(path);
                    }
                }
            }
        }
    }
}

size_t IteratedLocalSearch::DetermineLowerBoundVehicles()
{
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();

    // std::map<std::tuple<int, int>, int> infeasibleCombinations;

    std::vector<Container> containers;
    containers.reserve(mInstance->Vehicles.size());
    for (const auto& vehicle: mInstance->Vehicles)
    {
        containers.emplace_back(vehicle.Containers[0]);
    }

    std::vector<Node> customerNodes;
    customerNodes.reserve(mInstance->GetCustomers().size());
    for (const auto& node: mInstance->GetCustomers())
    {
        customerNodes.emplace_back(node);
    }

    auto groups = InterfaceConversions::NodesToGroup(customerNodes);
    auto binPacking = BinPacking1D(mEnv, containers, groups, mOutputPath);

    auto lowerBound1D = static_cast<size_t>(mLoadingChecker->SolveBinPackingApproximation());

    if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
    {
        while (mInstance->Vehicles.size() > lowerBound1D)
        {
            mInstance->Vehicles.pop_back();
        }
    }

    mTimer.LowerBoundVehicles = std::chrono::system_clock::now() - start;

    return lowerBound1D;
}

void IteratedLocalSearch::Solve()
{
    mLogFile << "### START EXACT APPROACH ###\n";

    //Write input parameters to json file
    std::string parameterString = "Parameters-" + mInstance->Name;
    Serializer::WriteToJson(mInputParameters, mOutputPath, parameterString);

    //Test if one of the one customer routes is infeasible
    Initialize();

    //Sets weights or volumes to 0 depending on the loading problem variant
    AdaptWeightsVolumesToLoadingProblem();

    //Determine infeasible arcs and tail paths
    //InfeasibleArcProcedure();

    mInstance->LowerBoundVehicles = DetermineLowerBoundVehicles();

    StartSolutionProcedure();

    //Iterated Local Search
    mTimer.startMetaheuristicTime();
    if(mInputParameters.IteratedLocalSearch.RunLS){

       LocalSearch::RunLocalSearch(mInstance, mLoadingChecker.get(), &mInputParameters, mCurrentSolution);

       if(mCurrentSolution.Costs < mBestSolution.Costs){
            mBestSolution = mCurrentSolution;
            mSolutionTracker.UpdateBothSolutions(0.0, mBestSolution.Costs);
       }
    }


    int NoImpr = 0;
    double maxRuntime = mInputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::ILS);
    if(mInputParameters.IteratedLocalSearch.RunILS){
        while(mTimer.getElapsedTime() < maxRuntime){

            std::cout << "Run: " << mSolutionTracker.iterations << " - CurrentBest: " << mCurrentSolution.Costs << " - OverallBest: " << mBestSolution.Costs << std::endl;
 
            LocalSearch::RunPerturbation(mInstance, mLoadingChecker.get(), &mInputParameters, mCurrentSolution, mRNG);
            LocalSearch::RunLocalSearch(mInstance, mLoadingChecker.get(), &mInputParameters, mCurrentSolution);

            std::cout << "Check path by classifer: " << std::endl; 
            auto items = InterfaceConversions::SelectItems(mCurrentSolution.Routes[0].Sequence, mInstance->Nodes, false);
            auto y = mClassifier->classify(items, mCurrentSolution.Routes[0].Sequence, mInstance->Vehicles.front().Containers.front());
            std::cout << "Output: " << y << std::endl;
            
            mTimer.calculateElapsedTime();

            if(mCurrentSolution.Costs < mBestSolution.Costs){
                mSolutionTracker.UpdateBothSolutions(mTimer.getElapsedTime(), mCurrentSolution.Costs);
                mBestSolution = mCurrentSolution;
                NoImpr = 0;
            }else{
                ++NoImpr;
                mSolutionTracker.UpdateCurrSolution(mTimer.getElapsedTime(), mCurrentSolution.Costs);
            }
            if(NoImpr >= mInputParameters.IteratedLocalSearch.NoImprLimit){
                mCurrentSolution = mBestSolution;
                mSolutionTracker.UpdateCurrSolution(mTimer.getElapsedTime(), mCurrentSolution.Costs);
                NoImpr = 0;
            }
            ++mSolutionTracker.iterations;

        }
    }

    mTimer.calculateMetaHeuristicTime();
    
    //TODO Add useful metrics here and variable for K Swaps

    auto statistics = SolverStatistics(mTimer,
                                       mSolutionTracker,
                                       mInfeasibleArcs.size(),
                                       mInfeasibleTailPaths.size());

    std::string solutionStatisticsString = "SolutionStatistics-" + mInstance->Name;
    Serializer::WriteToJson(statistics, mOutputPath, solutionStatisticsString);

    //TODO Change back to best later! 
    OutputSolution final_outputSolution(mBestSolution, mInstance);
    DeterminePackingSolution(final_outputSolution);

    auto solFile = SolutionFile(mInputParameters, statistics, final_outputSolution);

    std::string solutionString = "Solution-" + mInstance->Name;
    Serializer::WriteToJson(solFile, mOutputPath, solutionString);

    WriteSolutionSolutionValidator(final_outputSolution);
}

void IteratedLocalSearch::DeterminePackingSolution(OutputSolution& outputSolution)
{
    //Check if costs are equal to calcualted costs
    auto costs_before = outputSolution.Costs;
    outputSolution.DetermineCosts(mInstance);
    auto epsilon = 1e-9;

    if(fabs(costs_before - outputSolution.Costs) >= epsilon)
    {
        throw std::runtime_error("Solution exits with costs of " + std::to_string(costs_before) +
                                 " but has real costs of " + std::to_string(outputSolution.Costs));
    }

    //outputSolution.NumberOfRoutes = outputSolution.Tours.size();
    for (size_t tourId = 0; tourId < outputSolution.Tours.size(); tourId++)
    {
        auto& tour = outputSolution.Tours[tourId];
        auto& route = tour.Route;
        auto& container = tour.Vehicle.Containers.front();
        Collections::IdVector stopIds;
        std::vector<Cuboid> selectedItems;
        auto totalWeight = 0.0;
        auto totalVolume = 0.0;

        for (size_t i = 0; i < route.size(); ++i)
        {
            auto& items = route[i].Items;
            totalWeight += route[i].TotalWeight;
            totalVolume += route[i].TotalVolume;
            stopIds.push_back(route[i].InternId);

            for (auto& item: items)
            {
                item.GroupId = route.size() - 1 - i;
                selectedItems.emplace_back(item);
            }
        }

        if (totalWeight > container.WeightLimit)
        {
            throw std::runtime_error("Route " + std::to_string(tourId) + tour.Print() + " with total weight "
                                     + std::to_string(totalWeight) + " exceeds weight limit "
                                     + std::to_string(container.WeightLimit));
        }

        if (totalVolume > container.Volume)
        {
            throw std::runtime_error("Route " + std::to_string(tourId) + tour.Print() + " with total volume "
                                     + std::to_string(totalVolume) + " exceeds volume limit "
                                     + std::to_string(container.Volume));
        }

        mLogFile << "Route " << std::to_string(tourId) + tour.Print()
                 << ": weight util " + std::to_string(totalWeight / container.WeightLimit)
                 << " | volume util " + std::to_string(totalVolume / container.Volume) << " | ";

        if (!mInputParameters.ContainerLoading.LoadingProblem.EnableThreeDimensionalLoading)
        {
            mLogFile << "\n";
            continue;
        }

        auto heuristicStatus = LoadingStatus::Infeasible;
        /*
        if (mInputParameters.IteratedLocalSearch.ActivateHeuristic)
        {
            heuristicStatus =
                mLoadingChecker->PackingHeuristic(PackingType::Complete, container, stopIds, selectedItems);
        }
        */
        std::string feasStatusHeur = heuristicStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusHeur << " with packing heuristic | ";

        double maxRuntime = mInputParameters.DetermineMaxRuntime(IteratedLocalSearchParams::CallType::Exact);
        auto exactStatus = mLoadingChecker->ConstraintProgrammingSolverGetPacking(
            PackingType::Complete, container, stopIds, selectedItems, maxRuntime);

        std::string feasStatusCP = exactStatus == LoadingStatus::FeasOpt ? "feasible" : "infeasible";
        mLogFile << feasStatusCP << " with CP model"
                 << "\n";

        // TODO: packing as return value of loading checker

        if (exactStatus == LoadingStatus::Infeasible)
        {
            throw std::runtime_error("Loading infeasible according to CP model.");
        }

        size_t cItems = 0;
        for (auto& stop: route)
        {
            for (auto& item: stop.Items)
            {
                item = selectedItems[cItems];
                cItems++;
            }
        }
    }
}

void IteratedLocalSearch::PrintSolution(const OutputSolution& outputSolution)
{
    for (size_t tourId = 0; tourId < outputSolution.Tours.size(); tourId++)
    {
        const Tour& tour = outputSolution.Tours[tourId];
        const std::vector<Node>& route = tour.Route;

        mLogFile << "0 -> ";
        for (const auto& node: route)
        {
            mLogFile << node.InternId << " -> ";
        }
        mLogFile << "0"
                 << "\n";
    }
}

void IteratedLocalSearch::WriteSolutionSolutionValidator(const OutputSolution& outputSolution)
{
    ResultWriter::SolutionValidator::WriteInput(mOutputPath,
                                                mInstance->Name,
                                                InterfaceConversions::NodesToGroup(mInstance->Nodes),
                                                mInstance->Vehicles[0].Containers[0],
                                                mInstance->Vehicles.size());

    std::vector<std::vector<Group>> routes;
    routes.reserve(outputSolution.Tours.size());
    for (const auto& tour: outputSolution.Tours)
    {
        routes.emplace_back(InterfaceConversions::NodesToGroup(tour.Route));
    }

    ResultWriter::SolutionValidator::WriteOutput(mOutputPath, mInstance->Name, routes, outputSolution.Costs);
}

}
}