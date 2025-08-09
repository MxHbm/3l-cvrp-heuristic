#include "LoadingChecker/NoClassifierLoadingChecker.h"

namespace ContainerLoading
{
using namespace Algorithms;

bool NoClassifierLoadingChecker::CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items
                                    )
{
    if (RouteIsInFeasSequences(stopIds))
    {
        return true;
    }

    if (RouteIsInInfeasSequences(stopIds))
    {
        return false;
    }
    

    auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                            container,
                                            set,
                                            stopIds,
                                            items,
                                            false);

    return cpStatus == LoadingStatus::FeasOpt;

}
}

