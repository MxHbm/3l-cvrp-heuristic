// File: MLModelsContainer.h
#pragma once

#include <torch/script.h>
#include <vector>
#include <numeric> // for std::iota

#include "CommonBasics/Helper/ModelServices.h"
#include "Model/ContainerLoadingInstance.h"
#include "ProblemParameters.h"


//TODO Delete after check -- for saving tensor data!
/*
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>
*/
namespace ContainerLoading {

using namespace Model;
    
class Classifier {
public:

    Classifier(const ClassifierParams& classifierParams);

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    void saveClassifierResults(const std::vector<Cuboid>& items,
                                const Collections::IdVector& route,
                                const Container& container,
                                const float output,
                                const int status);

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    bool classify(const std::vector<Cuboid>& items,
                   const Collections::IdVector& route,
                   const Container& container);

    // Output: classification probability (0–1) - O Infeasible - 1 Feasible
    float classifyReturnOutput(const std::vector<Cuboid>& items,
                                const Collections::IdVector& route,
                                const Container& container);

    
    //Return bool value and write tensor data! 
    bool classifyWriteTensorData(const std::vector<Cuboid>& items,
                                        const Collections::IdVector& route,
                                        const Container& container,
                                        const int status);

private:
    
    torch::Tensor mean_tensor;
    torch::Tensor std_tensor;
    ClassifierParams mClassifierParams;
    torch::jit::script::Module model;

    void loadStandardScalingFromJson(const std::string& scaler_path);
    
    std::string get_timestamp();

    void save_tensor_to_csv(const torch::Tensor& tensor, const int status, const float output); 

    torch::Tensor applyStandardScaling(const torch::Tensor& input) const;

    torch::Tensor extractFeatures(const std::vector<Cuboid>& items,
                                  const Collections::IdVector& route,
                                  const Container& container) const;

    static float getMean(std::vector<float>::iterator first,
                        std::vector<float>::iterator last);

    static float getStd(std::vector<float>::iterator first,
                        std::vector<float>::iterator last);

};

}  // namespace ContianerLaoding

