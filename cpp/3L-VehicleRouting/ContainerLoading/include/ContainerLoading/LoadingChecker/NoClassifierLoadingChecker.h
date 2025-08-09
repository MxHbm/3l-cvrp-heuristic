#pragma once

#include "BaseLoadingChecker.h"
#include "Classifier.h"

namespace ContainerLoading
{
using namespace Algorithms;

class NoClassifierLoadingChecker : public BaseLoadingChecker
{
  public:

    using BaseLoadingChecker::BaseLoadingChecker; // inherits ctors

    [[nodiscard]] bool CompleteCheck(const Container& container,
                                    const boost::dynamic_bitset<>& set,
                                    const Collections::IdVector& stopIds,
                                    const std::vector<Cuboid>& items) override;

  private:
    std::unique_ptr<Classifier> mClassifier;
};
}