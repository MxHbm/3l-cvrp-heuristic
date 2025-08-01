#pragma once

#include "CommonBasics/Helper/ModelServices.h"

#include "ProblemParameters.h"

#include "Algorithms/MultiContainer/BP_MIP_1D.h"
#include "Model/ContainerLoadingInstance.h"
#include "Classifier.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/functional/hash.hpp>

#include <chrono>

namespace ContainerLoading
{
using namespace Algorithms;

class LoadingChecker
{
  public:
    const ContainerLoadingParams Parameters;

    explicit LoadingChecker(const ContainerLoadingParams& parameters, const double maxruntime) : Parameters(parameters), maxRunTime_CPSolver(maxruntime)
    {
        using enum LoadingFlag;

        std::vector<LoadingFlag> usedLoadingFlags = {Complete, NoSupport, LifoNoSequence};

        constexpr size_t reservedSize = 1000;
        for (const auto flag: usedLoadingFlags)
        {
            mFeasSequences[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mInfSequences[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mUnkSequences[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);

            mFeasibleSets[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mInfSets[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
            mUnknownSets[flag & Parameters.LoadingProblem.LoadingFlags].reserve(reservedSize);
        }

        mStartTime = std::chrono::high_resolution_clock::now();

         //Initialize Classifier: 
        if(Parameters.classifierParams.UseClassifier){
            mClassifier = std::make_unique<Classifier>(Parameters.classifierParams);
        }
    }

    [[nodiscard]] std::vector<Cuboid>
        SelectItems(const Collections::IdVector& nodeIds, std::vector<Group>& nodes, bool reversedDirection) const;

    [[nodiscard]] LoadingStatus ConstraintProgrammingSolver(PackingType packingType,
                                                            const Container& container,
                                                            const boost::dynamic_bitset<>& set,
                                                            const Collections::IdVector& stopIds,
                                                            const std::vector<Cuboid>& items,
                                                            bool isCallTypeExact);

    [[nodiscard]] LoadingStatus ConstraintProgrammingSolverGetPacking(PackingType packingType,
                                                                      const Container& container,
                                                                      const Collections::IdVector& stopIds,
                                                                      std::vector<Cuboid>& items,
                                                                      double maxRuntime) const;

    [[nodiscard]] bool CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items);

    void SetBinPackingModel(GRBEnv* env,
                            std::vector<Container>& containers,
                            std::vector<Group>& nodes,
                            const std::string& outputPath = "");

    [[nodiscard]] int SolveBinPackingApproximation() const;

    [[nodiscard]] int DetermineMinVehicles(bool enableLifting,
                                           double liftingThreshold,
                                           const Container& container,
                                           const boost::dynamic_bitset<>& nodes,
                                           double weight,
                                           double volume) const;

    [[nodiscard]] bool CustomerCombinationInfeasible(const boost::dynamic_bitset<>& customersInRoute) const;
    void AddInfeasibleCombination(const boost::dynamic_bitset<>& customersInRoute);
    [[nodiscard]] double GetElapsedTime();

    [[nodiscard]] Collections::SequenceVector GetFeasibleRoutes() const;
    [[nodiscard]] size_t GetNumberOfFeasibleRoutes() const;
    [[nodiscard]] size_t GetSizeInfeasibleCombinations() const;

    void AddFeasibleSequenceFromOutside(const Collections::IdVector& route);

    [[nodiscard]] bool RouteIsInFeasSequences(const Collections::IdVector& route) const;

    [[nodiscard]] bool RouteIsInInfeasSequences(const Collections::IdVector& route) const;

    [[nodiscard]] boost::dynamic_bitset<> MakeBitset(size_t size, const Collections::IdVector& sequence) const;

  private:
    std::chrono::high_resolution_clock::time_point mStartTime;
    std::unique_ptr<BinPacking1D> mBinPacking1D;
    std::unique_ptr<Classifier> mClassifier;
    const double maxRunTime_CPSolver;

    Collections::SequenceVector mCompleteFeasSeq;
    /// Set of customer combinations that are infeasible.
    /// -> There is no path in combination C that respects all constraints
    /// -> At least 2 vehicles are needed to serve all customers in C
    std::vector<boost::dynamic_bitset<>> mInfeasibleCustomerCombinations;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mFeasibleSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mFeasSequences;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mInfSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mInfSequences;

    std::unordered_map<LoadingFlag, std::vector<boost::dynamic_bitset<>>> mUnknownSets;
    std::unordered_map<LoadingFlag, Collections::SequenceSet> mUnkSequences;

    void AddFeasibleRoute(const Collections::IdVector& route);

    [[nodiscard]] bool SequenceIsInfeasibleCP(const Collections::IdVector& sequence, LoadingFlag mask) const;
    [[nodiscard]] bool SequenceIsUnknownCP(const Collections::IdVector& sequence, LoadingFlag mask) const;
    [[nodiscard]] bool SequenceIsFeasible(const Collections::IdVector& sequence, LoadingFlag mask) const;

    [[nodiscard]] bool SetIsInfeasibleCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;
    [[nodiscard]] bool SetIsUnknownCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;
    [[nodiscard]] bool SetIsFeasibleCP(const boost::dynamic_bitset<>& set, LoadingFlag mask) const;

    [[nodiscard]] LoadingFlag BuildMask(PackingType type) const;

    [[nodiscard]] LoadingStatus GetPrecheckStatusCP(const Collections::IdVector& sequence,
                                                    const boost::dynamic_bitset<>& set,
                                                    LoadingFlag mask,
                                                    bool isCallTypeExact);

    void AddStatus(const Collections::IdVector& sequence,
                   const boost::dynamic_bitset<>& set,
                   LoadingFlag mask,
                   LoadingStatus status);

    [[nodiscard]] int DetermineMinVehiclesBinPacking(bool enableLifting,
                                                     double liftingThreshold,
                                                     const boost::dynamic_bitset<>& nodes,
                                                     int r,
                                                     double z) const;

    [[nodiscard]] int ReSolveBinPackingApproximation(const boost::dynamic_bitset<>& selectedGroups) const;
};
}