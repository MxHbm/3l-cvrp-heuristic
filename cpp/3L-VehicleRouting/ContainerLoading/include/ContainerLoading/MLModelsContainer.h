// File: MLModelsContainer.h
#pragma once

#include <torch/script.h>
#include "CommonBasics/Helper/ModelServices.h"
#include "Model/ContainerLoadingInstance.h"
#include <vector>

namespace ContainerLoading {

using namespace Model;

namespace Classifier {
    
class MLModelsContainer {
public:

    MLModelsContainer(const std::string& model_path,
                      const std::string& scaler_path);

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
                        std::vector<float>::iterator last,
                        const int value);

    static float getStd(std::vector<float>::iterator first,
                        std::vector<float>::iterator last,
                        const int value);

    static void iota_own(std::vector<int>::iterator first,
                        std::vector<int>::iterator last,
                        const int value);

};

}  // namespace Classifier
}  // namespace ContainerLoading

