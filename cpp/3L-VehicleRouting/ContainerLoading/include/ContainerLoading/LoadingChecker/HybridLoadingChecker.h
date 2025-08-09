#pragma once

#include "BaseLoadingChecker.h"
#include "Classifier.h"

namespace ContainerLoading
{
using namespace Algorithms;

class HybridLoadingChecker : public BaseLoadingChecker
{
  public:

    explicit HybridLoadingChecker(const ContainerLoadingParams& parameters, const double maxruntime)
    : BaseLoadingChecker(parameters,maxruntime), mClassifier(std::make_unique<Classifier>(Parameters))
    {}

    [[nodiscard]] bool CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items) override;

  private:
    std::unique_ptr<Classifier> mClassifier;
};
}