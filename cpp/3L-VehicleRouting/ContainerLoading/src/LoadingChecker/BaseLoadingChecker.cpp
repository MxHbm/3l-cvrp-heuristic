#include "LoadingChecker/BaseLoadingChecker.h"
#include "Algorithms/SingleContainer/OPP_CP_3D.h"

namespace ContainerLoading
{
std::vector<Cuboid> BaseLoadingChecker::SelectItems(const Collections::IdVector& nodeIds,
                                                std::vector<Group>& nodes,
                                                bool reversedDirection) const
{
    std::vector<Cuboid> selectedItems;
    selectedItems.reserve(nodes.size() * 3);
    if (!reversedDirection)
    {
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            auto& items = nodes[nodeIds[i]].Items;

            for (auto& item: items)
            {
                item.GroupId = nodeIds.size() - 1 - i;
                selectedItems.push_back(item);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            auto& items = nodes[nodeIds[i]].Items;

            for (auto& item: items)
            {
                item.GroupId = i;
                selectedItems.push_back(item);
            }
        }
    }

    return selectedItems;
}

LoadingStatus BaseLoadingChecker::ConstraintProgrammingSolver(PackingType packingType,
                                                          const Container& container,
                                                          const boost::dynamic_bitset<>& set,
                                                          const Collections::IdVector& stopIds,
                                                          const std::vector<Cuboid>& items,
                                                          bool isCallTypeExact)
{
    auto loadingMask = BuildMask(packingType);

    auto precheckStatus = GetPrecheckStatusCP(stopIds, set, loadingMask, isCallTypeExact);
    if (precheckStatus != LoadingStatus::Invalid)
    {
        return precheckStatus;
    }

    auto numberStops = stopIds.size();
    auto containerLoadingCP = ContainerLoadingCP(Parameters,
                                                 container,
                                                 items,
                                                 numberStops,
                                                 loadingMask,
                                                 Parameters.SupportArea,
                                                 maxRunTime_CPSolver);

    auto status = containerLoadingCP.Solve();

    if (status == LoadingStatus::Invalid)
    {
        throw std::runtime_error("Loading status invalid in CP model!");
    }

    if (isCallTypeExact && status == LoadingStatus::Unknown)
    {
        return LoadingStatus::Invalid;
    }

    AddStatus(stopIds, set, loadingMask, status);

    return status;
}

LoadingStatus BaseLoadingChecker::ConstraintProgrammingSolverGetPacking(PackingType packingType,
                                                                    const Container& container,
                                                                    const Collections::IdVector& stopIds,
                                                                    std::vector<Cuboid>& items,
                                                                    double maxRuntime) const
{
    if (maxRuntime < 0.0 + 1e-5)
    {
        return LoadingStatus::Invalid;
    }

    auto loadingMask = BuildMask(packingType);

    auto numberStops = stopIds.size();

    auto containerLoadingCP = ContainerLoadingCP(Parameters,
                                                 container,
                                                 items,
                                                 numberStops,
                                                 loadingMask,
                                                 Parameters.SupportArea,
                                                 maxRuntime);

    auto status = containerLoadingCP.Solve();

    if (status == LoadingStatus::Invalid)
    {
        throw std::runtime_error("Loading status invalid in CP model!");
    }

    if (status == LoadingStatus::FeasOpt)
    {
        containerLoadingCP.ExtractPacking(items);
    }

    return status;
}

void BaseLoadingChecker::SetBinPackingModel(GRBEnv* env,
                                        std::vector<Container>& containers,
                                        std::vector<Group>& nodes,
                                        const std::string& outputPath)
{
    mBinPacking1D = std::make_unique<BinPacking1D>(env, containers, nodes, outputPath);
}

int BaseLoadingChecker::SolveBinPackingApproximation() const { return mBinPacking1D->Solve(); }

int BaseLoadingChecker::ReSolveBinPackingApproximation(const boost::dynamic_bitset<>& selectedGroups) const
{
    return mBinPacking1D->ReSolve(selectedGroups);
}

int BaseLoadingChecker::DetermineMinVehicles(bool enableLifting,
                                         double liftingThreshold,
                                         const Container& container,
                                         const boost::dynamic_bitset<>& nodes,
                                         double weight,
                                         double volume) const
{
    auto weightRatio = container.WeightLimit > 1 ? weight / container.WeightLimit : 0.0;
    auto volumeRatio = container.Volume > 1 ? volume / container.Volume : 0.0;

    auto z = std::max(weightRatio, volumeRatio);
    auto r = static_cast<int>(std::ceil(z));

    int minVehicles = DetermineMinVehiclesBinPacking(enableLifting, liftingThreshold, nodes, r, z);

    return minVehicles;
}

int BaseLoadingChecker::DetermineMinVehiclesBinPacking(bool enableLifting,
                                                   double liftingThreshold,
                                                   const boost::dynamic_bitset<>& nodes,
                                                   int r,
                                                   double z) const
{
    if (!enableLifting || r == 1 || r - z > liftingThreshold)
    {
        return r;
    }

    int minVehicles = ReSolveBinPackingApproximation(nodes);

    ////if (minVehicles > r)
    ////    std::cout << "1DBP: " << minVehicles << " | " << r << "\n";

    return minVehicles;
}

bool BaseLoadingChecker::CustomerCombinationInfeasible(const boost::dynamic_bitset<>& customersInRoute) const
{
    for (const auto& customerCombination: mInfeasibleCustomerCombinations)
    {
        auto intersection = customersInRoute & customerCombination;

        if (intersection.count() != customerCombination.count())
        {
            continue;
        }

        return true;
    }

    return false;
}


void BaseLoadingChecker::AddInfeasibleCombination(const boost::dynamic_bitset<>& customersInRoute)
{
    mInfeasibleCustomerCombinations.emplace_back(customersInRoute);
}

Collections::SequenceVector BaseLoadingChecker::GetFeasibleRoutes() const { return mCompleteFeasSeq; };

size_t BaseLoadingChecker::GetNumberOfFeasibleRoutes() const { return mCompleteFeasSeq.size(); };

size_t BaseLoadingChecker::GetSizeInfeasibleCombinations() const { return mInfeasibleCustomerCombinations.size(); };

void BaseLoadingChecker::AddFeasibleSequenceFromOutside(const Collections::IdVector& route) { AddFeasibleRoute(route); }

bool BaseLoadingChecker::RouteIsInFeasSequences(const Collections::IdVector& route) const
{
    return mFeasSequences.at(Parameters.LoadingFlags).contains(route);
}

bool BaseLoadingChecker::RouteIsInInfeasSequences(const Collections::IdVector& route) const
{
    return mInfSequences.at(Parameters.LoadingFlags).contains(route);
}

boost::dynamic_bitset<> BaseLoadingChecker::MakeBitset(size_t size, const Collections::IdVector& sequence) const
{
    boost::dynamic_bitset<> set(size);
    for (const auto i: sequence)
    {
        set.set(i);
    }

    return set;
};

void BaseLoadingChecker::AddFeasibleRoute(const Collections::IdVector& route)
{
    mFeasSequences[Parameters.LoadingFlags].insert(route);
    mCompleteFeasSeq.push_back(route);
}

bool BaseLoadingChecker::SequenceIsInfeasibleCP(const Collections::IdVector& sequence, const LoadingFlag mask) const
{
    return mInfSequences.at(mask).contains(sequence);
}

bool BaseLoadingChecker::SequenceIsUnknownCP(const Collections::IdVector& sequence, const LoadingFlag mask) const
{
    return mUnkSequences.at(mask).contains(sequence);
}

bool BaseLoadingChecker::SequenceIsFeasible(const Collections::IdVector& sequence, const LoadingFlag mask) const
{
    return mFeasSequences.at(mask).contains(sequence);
}

bool BaseLoadingChecker::SetIsInfeasibleCP(const boost::dynamic_bitset<>& set, const LoadingFlag mask) const
{
    const auto& sets = mInfSets.at(mask);
    if (!IsSet(mask, LoadingFlag::Support))
    {
        // If support is disabled, set S is infeasible when S is a superset of an infeasible set.
        auto setComparer = [set](const boost::dynamic_bitset<>& infeasibleSet)
        { return (set & infeasibleSet).count() == infeasibleSet.count(); };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }
    else
    {
        // If support is enabled, only exact matching of sets can be used as adding additional items can lead to
        // feasibility.
        auto setComparer = [set](const boost::dynamic_bitset<>& feasibleCombination)
        { return set == feasibleCombination; };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }

    return false;
}

bool BaseLoadingChecker::SetIsUnknownCP(const boost::dynamic_bitset<>& set, const LoadingFlag mask) const
{
    const auto& sets = mUnknownSets.at(mask);

    auto setComparer = [set](const boost::dynamic_bitset<>& feasibleCombination) { return set == feasibleCombination; };

    if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
    {
        return true;
    }

    return false;
}

bool BaseLoadingChecker::SetIsFeasibleCP(const boost::dynamic_bitset<>& set, const LoadingFlag mask) const
{
    const auto& sets = mFeasibleSets.at(mask);
    if (!IsSet(mask, LoadingFlag::Support))
    {
        // If support is disabled, set S is feasible when S is a subset of a feasible set.
        auto setComparer = [set](const boost::dynamic_bitset<>& feasibleSet)
        { return (set & feasibleSet).count() == set.count(); };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }
    else
    {
        // If support is enabled, only exact matching of sets can be used as removing items can lead to infeasibility.
        auto setComparer = [set](const boost::dynamic_bitset<>& feasibleCombi) { return set == feasibleCombi; };

        if (std::find_if(std::begin(sets), std::end(sets), setComparer) != std::end(sets))
        {
            return true;
        }
    }

    return false;
}

LoadingFlag BaseLoadingChecker::BuildMask(PackingType type) const
{
    switch (type)
    {
        case PackingType::Complete:
            return LoadingFlag::Complete &Parameters.LoadingFlags;
        case PackingType::NoSupport:
            return LoadingFlag::NoSupport &Parameters.LoadingFlags;
        case PackingType::LifoNoSequence:
            return LoadingFlag::LifoNoSequence &Parameters.LoadingFlags;
        default:
            throw std::runtime_error("PackingType not implemented in mask builder.");
    }

    return LoadingFlag();
}

LoadingStatus BaseLoadingChecker::GetPrecheckStatusCP(const Collections::IdVector& sequence,
                                                  const boost::dynamic_bitset<>& set,
                                                  const LoadingFlag mask,
                                                  const bool isCallTypeExact)
{
    if (IsSet(mask, LoadingFlag::Sequence))
    {
        if (SequenceIsInfeasibleCP(sequence, mask))
        {
            ////std::cout << "Sequence already stored as infeasible (CP)." << "\n";
            return LoadingStatus::Infeasible;
        }

        if (SequenceIsFeasible(sequence, mask))
        {
            ////std::cout << "Sequence already stored as feasible (CP)." << "\n";
            return LoadingStatus::FeasOpt;
        }

        if (!isCallTypeExact && SequenceIsUnknownCP(sequence, mask))
        {
            ////std::cout << "Sequence already stored as unknown (CP)." << "\n";
            return LoadingStatus::Unknown;
        }
    }
    else
    {
        if (SetIsInfeasibleCP(set, mask))
        {
            ////std::cout << "Set already stored as infeasible (CP)." << "\n";
            return LoadingStatus::Infeasible;
        }

        if (SetIsFeasibleCP(set, mask))
        {
            ////std::cout << "Set already stored as feasible (CP)." << "\n";
            if (mask ==Parameters.LoadingFlags && !SequenceIsFeasible(sequence, mask))
            {
                AddFeasibleRoute(sequence);
            }

            return LoadingStatus::FeasOpt;
        }

        if (!isCallTypeExact && SetIsUnknownCP(set, mask))
        {
            ////std::cout << "Set already stored as unknown (CP)." << "\n";
            return LoadingStatus::Unknown;
        }
    }

    return LoadingStatus::Invalid;
}

void BaseLoadingChecker::AddStatus(const Collections::IdVector& sequence,
                               const boost::dynamic_bitset<>& set,
                               const LoadingFlag mask,
                               const LoadingStatus status)
{
    // Add to feasible sequences although lifo might be disabled; needed for SP heuristic.
    if (status == LoadingStatus::FeasOpt && mask ==Parameters.LoadingFlags)
    {
        AddFeasibleRoute(sequence);
        if (!IsSet(mask, LoadingFlag::Lifo))
        {
            mFeasibleSets[mask].push_back(set);
        }

        return;
    }

    // If lifo is enabled, order of stops is relevant -> sequence of ids.
    if (IsSet(mask, LoadingFlag::Sequence))
    {
        switch (status)
        {
            case LoadingStatus::FeasOpt:
            {
                mFeasSequences[mask].insert(sequence);
                return;
            }
            case LoadingStatus::Infeasible:
            {
                mInfSequences[mask].insert(sequence);
                return;
            }
            case LoadingStatus::Unknown:
            {
                mUnkSequences[mask].insert(sequence);
                return;
            }
            default:
                throw std::runtime_error("LoadingStatus invalid!");
        }
    }
    // If lifo is disabled, order of stops is not relevant -> set of ids.
    else
    {
        switch (status)
        {
            case LoadingStatus::FeasOpt:
            {
                mFeasibleSets[mask].push_back(set);
                return;
            }
            case LoadingStatus::Infeasible:
            {
                mInfSets[mask].push_back(set);
                return;
            }
            case LoadingStatus::Unknown:
            {
                mUnknownSets[mask].push_back(set);
                return;
            }
            default:
                throw std::runtime_error("LoadingStatus invalid!");
        }
    }
}

}