// File: MLModelsContainer.h
#pragma once

#include <torch/script.h>
#include <vector>
#include <numeric> // for std::iota


#include "CommonBasics/Helper/ModelServices.h"
#include "Model/ContainerLoadingInstance.h"
#include "ProblemParameters.h"

namespace ContainerLoading {

using namespace Model;
    
class Classifier {
public:

    Classifier(const ClassifierParams& classifierParams);

    // Output: classification probability (0–1)
    float classify(const std::vector<Cuboid>& items,
                   const Collections::IdVector& route,
                   const Container& container);

private:
    
    torch::Tensor mean_tensor;
    torch::Tensor std_tensor;

    void loadStandardScalingFromJson(const std::string& scaler_path);

    torch::Tensor applyStandardScaling(const torch::Tensor& input) const;

    torch::jit::script::Module model;
    torch::Tensor extractFeatures(const std::vector<Cuboid>& items,
                                  const Collections::IdVector& route,
                                  const Container& container) const;

    static float getMean(std::vector<float>::iterator first,
                        std::vector<float>::iterator last);

    static float getStd(std::vector<float>::iterator first,
                        std::vector<float>::iterator last);

};

}  // namespace ContianerLaoding

