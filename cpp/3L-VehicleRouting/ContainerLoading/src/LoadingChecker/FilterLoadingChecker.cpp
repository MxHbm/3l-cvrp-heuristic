#include "LoadingChecker/FilterLoadingChecker.h"

namespace ContainerLoading
{
using namespace Algorithms;

bool FilterLoadingChecker::CompleteCheck(const Container& container,
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
    
        
        if(mClassifier->classify(items,stopIds,container)){

        auto cpStatus = ConstraintProgrammingSolver(PackingType::Complete,
                                                container,
                                                set,
                                                stopIds,
                                                items,
                                                false);

        return cpStatus == LoadingStatus::FeasOpt;

    }else{
        return false;
    }
}

}