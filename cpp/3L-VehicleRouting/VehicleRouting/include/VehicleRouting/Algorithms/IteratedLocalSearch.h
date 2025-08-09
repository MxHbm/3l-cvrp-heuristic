#pragma once

#include "ContainerLoading/LoadingChecker/BaseLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/FilterLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/HybridLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/NoClassifierLoadingChecker.h"
#include "ContainerLoading/LoadingChecker/SpeedUpLoadingChecker.h"
#include "ContainerLoading/Algorithms/CPSolverParameters.h"
#include "ContainerLoading/Algorithms/LoadingStatus.h"
#include "ContainerLoading/Helper/HelperIO.h"
#include "Improvement/LocalSearch.h"
#include "Algorithms/Evaluation.h"
#include "Algorithms/LoadingInterfaceServices.h"
#include "Algorithms/Constructive.h"
#include "CommonBasics/Helper/ModelServices.h"
#include "Helper/HelperIO.h"
#include "Helper/Serialization.h"
#include "Helper/Timer.h"
#include "Model/Instance.h"
#include "Model/Solution.h"

#include <fstream>
#include <iostream>
#include <random>
#include <cstdint>
#include <memory>



namespace VehicleRouting
{
using namespace Model;
using namespace Helper;

namespace Algorithms
{
using namespace ContainerLoading;
using namespace ContainerLoading::Model;

class IteratedLocalSearch
{
  public:
  IteratedLocalSearch(Instance* instance,
                      GRBEnv* env,
                      const VehicleRouting::InputParameters& inputParameters,
                      const std::string& startSolutionFolderPath,
                      const std::string& outputPath,
                      const int seedOffset)
    : mEnv(env),
      mInstance(instance),
      mInputParameters(inputParameters),
      mStartSolutionFolderPath(startSolutionFolderPath),
      mOutputPath(outputPath),
      mSeedOffset(seedOffset),
      mSolutionTracker(mSeedOffset)
    {
        mLogFile.open(env->get(GRB_StringParam_LogFile), std::ios::out | std::ios::app);

        //Initialize Local Search
        mLocalSearch = std::make_unique<Improvement::LocalSearch>(mInputParameters, mInstance);

        //Initialize RNG 
        mRNG.seed(42 + seedOffset);
    }

    void Solve();

  private:
    GRBEnv* mEnv;
    Instance* mInstance;
    InputParameters mInputParameters;
    std::string mStartSolutionFolderPath;
    std::string mOutputPath;
    int mSeedOffset;

    std::ofstream mLogFile;
    std::vector<Arc> mInfeasibleArcs;
    std::vector<Arc> mInfeasibleTailPaths;
    Collections::SequenceSet mInfeasibleCombinations;
    //std::vector<Arc> mCurrentSolutionArcs;

    Solution mCurrentSolution;
    Solution mBestSolution;

    SolutionTracker mSolutionTracker;

    std::mt19937 mRNG;

    std::unique_ptr<BaseLoadingChecker> mLoadingChecker;
    std::unique_ptr<Improvement::LocalSearch> mLocalSearch;

    void InfeasibleArcProcedure();
    void DetermineInfeasiblePaths();
    bool CheckPath(const Collections::IdVector& path, Container& container, std::vector<Cuboid>& items);
    void DetermineExtendedInfeasiblePath();
    void DetermineInfeasibleCustomerCombinations();

    Helper::Timer mTimer = Helper::Timer();

    size_t DetermineLowerBoundVehicles();

    void Initialize();
    void TestProcedure();
    void AdaptWeightsVolumesToLoadingProblem();
    void DeterminePackingSolution(OutputSolution& outputSolution);
    void PrintSolution(const OutputSolution& outputSolution);

    void WriteSolutionSolutionValidator(const OutputSolution& outputSolution);

    void StartSolutionProcedure();
    void GenerateStartSolutionSavings();
    void GenerateStartSolutionModifiedSavings();
    void GenerateStartSolutionSPHeuristic();
      /*
    bool IsCurrentSolutionCPValid(const Solution& solution, double time_limit);

    bool IteratedLocalSearch::IsCurrentSolutionCPValid(const Solution& solution, double time_limit) {
        for(const auto& route : solution.Routes) {

            if(route.Sequence.size() > 0){

                auto items = InterfaceConversions::SelectItems(route.Sequence, mInstance->Nodes, false);
                auto status =
                    mLoadingChecker->HeuristicCompleteCheck(mInstance->Vehicles.front().Containers.front(),
                                                            mLoadingChecker->MakeBitset(mInstance->Nodes.size(), route.Sequence),
                                                            route.Sequence,
                                                            items,
                                                            time_limit);

                if(status != LoadingStatus::FeasOpt) {
                    //std::cout << "Route was rejected by CPSolver" << std::endl;
                    ++mSolutionTracker.rejections;
                    return false;
                }
            }
        }
        return true;
    }
    */


};
}
}