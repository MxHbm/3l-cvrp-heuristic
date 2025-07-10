// File: MLModelsContainer.cpp

#include "MLModelsContainer.h"
#include <cmath>
#include <fstream>
#include "nlohmann/json.hpp"

namespace ContainerLoading {
namespace Classifier {

void MLModelsContainer::loadStandardScalingFromJson(const std::string& scaler_path){

    std::ifstream file(scaler_path);
    if (!file) {
        throw std::runtime_error("Could not open scaling JSON file.");
    }

    nlohmann::json j;
    file >> j;

    std::vector<double> mean_vec = j["mean"];
    std::vector<double> std_vec = j["std"];

    mean_tensor = torch::tensor(mean_vec, torch::kFloat32).unsqueeze(0); // shape: [1, N]
    std_tensor = torch::tensor(std_vec, torch::kFloat32).unsqueeze(0);   // shape: s[1, N]
}

MLModelsContainer::MLModelsContainer(const ClassifierParams& classifierParams){

    model = torch::jit::load(classifierParams.TracedModelPath);
    model.eval();

    loadStandardScalingFromJson(classifierParams.SerializeJson_MeanStd);

}

torch::Tensor MLModelsContainer::applyStandardScaling(const torch::Tensor& input) const{

    return (input - mean_tensor) / std_tensor;
}



void MLModelsContainer::iota_own(std::vector<int>::iterator first,
                                 std::vector<int>::iterator last,
                                 const int value)
{
    auto mid = -(value / 2); 

    if(value % 2 != 0){
        for (; first != last; ++first, ++mid)
            *first = mid;
    }else{
        for (; first != last; ++first, ++mid)
            if(mid == 0){
                ++mid;
            }
            *first = mid;
    }
}

float MLModelsContainer::getMean(std::vector<float>::iterator first,
                                 std::vector<float>::iterator last,
                                 const int noItems){

    auto init{0.0f};                                
    for (; first != last; ++first)
        init += *first;
 
    return init / noItems;
}

float MLModelsContainer::getStd(std::vector<float>::iterator first,
                                 std::vector<float>::iterator last,
                                 const int noItems){

    if(noItems == 1){
        return 0.0f;
    }

    auto const mean = getMean(first, last, noItems); 
    
    auto init{0.0f};    
    for (; first != last; ++first)
        init += std::pow(*first - mean, 2);
 
    return std::sqrt(init / noItems);

}



//['NoItems', 'NoCustomers', 'Rel Volume', 'Rel Weight', 'Weight Distribution', 'Volume Distribution', 'Fragile Ratio',
//'Rel Total Length Items', 'Rel Total Width Items', 'Rel Total Height Items']

torch::Tensor MLModelsContainer::extractFeatures(const std::vector<Cuboid>& items,
                                                const Collections::IdVector& route,
                                                const Container& container) const {

    torch::Tensor result = torch::zeros({1,38});
    //std::vector<float> features;
    //features.reserve(38);
    
    const auto containerWeightLimit = container.WeightLimit;
    const auto containerVolume = container.Volume;
    const auto noItems = items.size();
    const auto noCustomers = route.size();
    const auto containerDx = container.Dx;
    const auto containerDy = container.Dy;
    const auto containerDz = container.Dz;

    //No Items
    result[0][0] = static_cast<float>(noItems);

    //NoCustomers
    result[0][1] = static_cast<float>(noCustomers);
    
    std::vector<int> pyramideValues;
    pyramideValues.reserve(noCustomers); 
    iota_own(pyramideValues.begin(), pyramideValues.end(), noCustomers);

	std::vector<float> width_height_ratios(noItems, 0.0f);
    std::vector<float> length_height_ratios(noItems, 0.0f);
    std::vector<float> width_length_ratios(noItems, 0.0f);
    std::vector<float> length_L_ratios(noItems, 0.0f);
    std::vector<float> width_W_ratios(noItems, 0.0f);
    std::vector<float> height_H_ratios(noItems, 0.0f);
    std::vector<float> volume_WLH_ratios(noItems, 0.0f);

    auto tot_volume = 0.0f;
    auto tot_weight = 0.0f;
    auto fragile_count = 0.0f;
    auto tot_length = 0.0f;
    auto tot_width = 0.0f; 
    auto tot_height = 0.0f; 
    auto volumeDistribution= 0.0f;
    auto weightDistribution = 0.0f;

    int it = 0;
    for (const auto& item : items) {
        // Extract features per Rectangle, e.g.:
        tot_volume += item.Volume;
        tot_weight += item.Weight;
        tot_width += item.Dx;
        tot_length += item.Dy;
        tot_height += item.Dz;
        if(item.Fragility == Fragility::Fragile){
            ++fragile_count;
        }
        //Distributions
        volumeDistribution += item.Volume * pyramideValues[item.GroupId];
        weightDistribution += item.Weight * pyramideValues[item.GroupId];

        // Add more as needed, matching the Python training input

        width_height_ratios[it] = item.Dx / item.Dz;
        length_height_ratios[it] = item.Dy / item.Dz;
        width_length_ratios[it] = item.Dx / item.Dy;
        length_L_ratios[it] = item.Dy / containerDy;
        width_W_ratios[it] = item.Dx / containerDx;
        height_H_ratios[it] = item.Dz / containerDz;
        volume_WLH_ratios[it] = item.Volume / containerVolume;

        ++it;
    }

    //'Rel Volume' and 'Rel Weight'
    result[0][2] = tot_volume / containerVolume;
    result[0][3] = tot_weight / containerWeightLimit;

    //'Weight Distribution', 'Volume Distribution'
    result[0][4] = weightDistribution / containerWeightLimit;
    result[0][5] = volumeDistribution / containerVolume;

    //'Fragile Ratio'
    result[0][6] = fragile_count / noItems;

    //Rel Total Length Items', 'Rel Total Width Items', 'Rel Total Height Items', 
    result[0][7] = tot_length / containerDy;
    result[0][8] = tot_width / containerDx;
    result[0][9] = tot_height / containerDz;

    // 'width_height_min', 'width_height_max', 'width_height_mean', 'width_height_std',
    result[0][10] = *std::min_element(width_height_ratios.begin(), width_height_ratios.end());
    result[0][11] = *std::max_element(width_height_ratios.begin(), width_height_ratios.end());
    result[0][12] = getMean(width_height_ratios.begin(), width_height_ratios.end(), noItems);
    result[0][13] = getStd(width_height_ratios.begin(), width_height_ratios.end(), noItems);
    
    //'length_height_min', 'length_height_max', 'length_height_mean', 'length_height_std',
    result[0][14] = *std::min_element(length_height_ratios.begin(), length_height_ratios.end());
    result[0][15] = *std::max_element(length_height_ratios.begin(), length_height_ratios.end());
    result[0][16] = getMean(length_height_ratios.begin(), length_height_ratios.end(), noItems);
    result[0][17] = getStd(length_height_ratios.begin(), length_height_ratios.end(), noItems);

    // 'width_length_min', 'width_length_max', 'width_length_mean', 'width_length_std',
    result[0][18] = *std::min_element(width_length_ratios.begin(), width_length_ratios.end());
    result[0][19] = *std::max_element(width_length_ratios.begin(), width_length_ratios.end());
    result[0][20] = getMean(width_length_ratios.begin(), width_length_ratios.end(), noItems);
    result[0][21] = getStd(width_length_ratios.begin(), width_length_ratios.end(), noItems);

    // 'width_W_min', 'width_W_max', 'width_W_mean', 'width_W_std', 
    result[0][22] = *std::min_element(width_W_ratios.begin(), width_W_ratios.end());
    result[0][23] = *std::max_element(width_W_ratios.begin(), width_W_ratios.end());
    result[0][24] = getMean(width_W_ratios.begin(), width_W_ratios.end(), noItems);
    result[0][25] = getStd(width_W_ratios.begin(), width_W_ratios.end(), noItems);
    
    //'length_L_min', 'length_L_max', 'length_L_mean', 'length_L_std',
    result[0][26] = *std::min_element(length_L_ratios.begin(), length_L_ratios.end());
    result[0][27] = *std::max_element(length_L_ratios.begin(), length_L_ratios.end());
    result[0][28] = getMean(length_L_ratios.begin(), length_L_ratios.end(), noItems);
    result[0][29] = getStd(length_L_ratios.begin(), length_L_ratios.end(), noItems);
    
    //'height_H_min', 'height_H_max', 'height_H_mean', 'height_H_std'
    result[0][30] = *std::min_element(height_H_ratios.begin(), height_H_ratios.end());
    result[0][31] = *std::max_element(height_H_ratios.begin(), height_H_ratios.end());
    result[0][32] = getMean(height_H_ratios.begin(), height_H_ratios.end(), noItems);
    result[0][33] = getStd(height_H_ratios.begin(), height_H_ratios.end(), noItems);
    
    //'volume_WLH_min', 'volume_WLH_max', 'volume_WLH_mean', 'volume_WLH_std'
    result[0][34] = *std::min_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][35] = *std::max_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][36] = getMean(volume_WLH_ratios.begin(), volume_WLH_ratios.end(), noItems);
    result[0][37] = getStd(volume_WLH_ratios.begin(), volume_WLH_ratios.end(), noItems);

    // Resize or pad to match model input if needed
    return result;
}

float MLModelsContainer::classify(const std::vector<Cuboid>& items,
                                  const Collections::IdVector& route,
                                  const Container& container) {

    torch::Tensor input = extractFeatures(items, route, container);
    // Apply scaling before inference
    torch::Tensor input_scaled = applyStandardScaling(input);
    torch::Tensor output = model.forward({input_scaled}).toTensor();
    return output.item<float>();
}

}  // namespace Classifier
}  // namespace ContainerLoading
