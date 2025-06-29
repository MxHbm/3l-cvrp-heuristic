#pragma once

#include "Model/Instance.h"
//TODO Delete afterwards
#include <iostream>

namespace VehicleRouting
{
using namespace Model;
namespace Algorithms
{

class Evaluator
{
  public:
    static double CalculateRouteCosts(const Instance* const instance, const Collections::IdVector& route)
    {
        double costs = 0.0;

        costs += instance->Distance(instance->GetDepotId(), route.front());

        for (size_t iNode = 0; iNode < route.size() - 1; iNode++)
        {
            costs += instance->Distance(route[iNode], route[iNode + 1]);
        }

        costs += instance->Distance(route.back(), instance->GetDepotId());

        return costs;
    };

    static double CalculateRouteCosts(const Instance* const instance, const std::vector<Node>& route)
    {
        double costs = 0.0;

        costs += instance->Distance(instance->GetDepotId(), route.front().InternId);

        for (size_t iNode = 0; iNode < route.size() - 1; iNode++)
        {
            costs += instance->Distance(route[iNode].InternId, route[iNode + 1].InternId);
        }

        costs += instance->Distance(route.back().InternId, instance->GetDepotId());

        return costs;
    };

    static double CalculateInsertionCosts(const Instance* const instance, size_t tailId, size_t headId, size_t nodeId)
    {
        return instance->Distance(tailId, nodeId) + instance->Distance(nodeId, headId)
               - instance->Distance(tailId, headId);
    }

    static double CalculateSavings(const Instance* const instance, size_t tailId, size_t headId)
    {
        return instance->Distance(tailId, headId)
               - (instance->Distance(tailId, instance->GetDepotId())
                  + instance->Distance(instance->GetDepotId(), headId));
    }

    static double CalculateInterSwapDelta(const Instance* const instance,
                                          const Collections::IdVector& routeA,
                                          const Collections::IdVector& routeB,
                                          const size_t nodeA,
                                          const size_t nodeB)
    {
        auto interID_A = routeA[nodeA];
        auto interID_B = routeB[nodeB];
        auto internID_precNodeA = (nodeA == 0) ? instance->GetDepotId() : routeA[nodeA - 1];
        auto internID_succNodeA = (nodeA == routeA.size() - 1) ? instance->GetDepotId() : routeA[nodeA + 1];
        auto internID_precNodeB = (nodeB == 0) ? instance->GetDepotId() : routeB[nodeB - 1];
        auto internID_succNodeB = (nodeB == routeB.size() - 1) ? instance->GetDepotId() : routeB[nodeB + 1];


        double savings = instance->Distance(internID_precNodeA, interID_B)
                         + instance->Distance(interID_B, internID_succNodeA)
                         + instance->Distance(internID_precNodeB, interID_A)
                         + instance->Distance(interID_A, internID_succNodeB)
                         - instance->Distance(internID_precNodeA, interID_A) 
                         - instance->Distance(interID_A, internID_succNodeA)
                         - instance->Distance(internID_precNodeB, interID_B)
                         - instance->Distance(interID_B, internID_succNodeB);

        return savings;
    }  
};

}
}