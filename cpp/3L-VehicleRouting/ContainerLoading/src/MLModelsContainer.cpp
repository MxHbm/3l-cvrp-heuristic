// File: MLModelsContainer.cpp

#include "MLModelsContainer.h"
#include <cmath>

namespace ContainerLoading {
namespace Classifier {

MLModelsContainer::MLModelsContainer(const std::string& model_path) {
    model = torch::jit::load(model_path);
    model.eval();
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
    std::vector<float> features;
    features.reserve(38);
    
    const auto containerWeightLimit = container.WeightLimit;
    const auto containerVolume = container.Volume;
    const auto noItems = items.size();
    const auto noCustomers = route.size();
    const auto containerDx = container.Dx;
    const auto containerDy = container.Dy;
    const auto containerDz = container.Dz;

    //No Items
    features.push_back(static_cast<float>(noItems));

    //NoCustomers
    features.push_back(static_cast<float>(noCustomers));
    
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

    auto tot_volume = 0;
    auto tot_weight = 0;
    auto fragile_count = 0;
    auto tot_length = 0;
    auto tot_width = 0; 
    auto tot_height = 0; 
    auto volumeDistribution{0.0f};
    auto weightDistribution{0.0f};

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
    features.push_back(static_cast<float>(tot_volume / containerVolume));
    features.push_back(static_cast<float>(tot_weight / containerWeightLimit));

    //'Weight Distribution', 'Volume Distribution'
    features.push_back(static_cast<float>(weightDistribution / containerWeightLimit));
    features.push_back(static_cast<float>(volumeDistribution / containerVolume));

    //'Fragile Ratio'
    features.push_back(static_cast<float>(fragile_count / noItems));

    //Rel Total Length Items', 'Rel Total Width Items', 'Rel Total Height Items', 
    features.push_back(static_cast<float>(tot_length / containerDy));
    features.push_back(static_cast<float>(tot_width / containerDx));
    features.push_back(static_cast<float>(tot_height / containerDz));

    // 'width_height_min', 'width_height_max', 'width_height_mean', 'width_height_std',
    features.push_back(*std::min_element(width_height_ratios.begin(), width_height_ratios.end()));
    features.push_back(*std::max_element(width_height_ratios.begin(), width_height_ratios.end()));
    features.push_back(getMean(width_height_ratios.begin(), width_height_ratios.end(), noItems));
    features.push_back(getStd(width_height_ratios.begin(), width_height_ratios.end(), noItems));
    
    //'length_height_min', 'length_height_max', 'length_height_mean', 'length_height_std',
    features.push_back(*std::min_element(length_height_ratios.begin(), length_height_ratios.end()));
    features.push_back(*std::max_element(length_height_ratios.begin(), length_height_ratios.end()));
    features.push_back(getMean(length_height_ratios.begin(), length_height_ratios.end(), noItems));
    features.push_back(getStd(length_height_ratios.begin(), length_height_ratios.end(), noItems));

    // 'width_length_min', 'width_length_max', 'width_length_mean', 'width_length_std',
    features.push_back(*std::min_element(width_length_ratios.begin(), width_length_ratios.end()));
    features.push_back(*std::max_element(width_length_ratios.begin(), width_length_ratios.end()));
    features.push_back(getMean(width_length_ratios.begin(), width_length_ratios.end(), noItems));
    features.push_back(getStd(width_length_ratios.begin(), width_length_ratios.end(), noItems));

    // 'width_W_min', 'width_W_max', 'width_W_mean', 'width_W_std', 
    features.push_back(*std::min_element(width_W_ratios.begin(), width_W_ratios.end()));
    features.push_back(*std::max_element(width_W_ratios.begin(), width_W_ratios.end()));
    features.push_back(getMean(width_W_ratios.begin(), width_W_ratios.end(), noItems));
    features.push_back(getStd(width_W_ratios.begin(), width_W_ratios.end(), noItems));
    
    //'length_L_min', 'length_L_max', 'length_L_mean', 'length_L_std',
    features.push_back(*std::min_element(length_L_ratios.begin(), length_L_ratios.end()));
    features.push_back(*std::max_element(length_L_ratios.begin(), length_L_ratios.end()));
    features.push_back(getMean(length_L_ratios.begin(), length_L_ratios.end(), noItems));
    features.push_back(getStd(length_L_ratios.begin(), length_L_ratios.end(), noItems));
    
    //'height_H_min', 'height_H_max', 'height_H_mean', 'height_H_std'
    features.push_back(*std::min_element(height_H_ratios.begin(), height_H_ratios.end()));
    features.push_back(*std::max_element(height_H_ratios.begin(), height_H_ratios.end()));
    features.push_back(getMean(height_H_ratios.begin(), height_H_ratios.end(), noItems));
    features.push_back(getStd(height_H_ratios.begin(), height_H_ratios.end(), noItems));
    
    //'volume_WLH_min', 'volume_WLH_max', 'volume_WLH_mean', 'volume_WLH_std'
    features.push_back(*std::min_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end()));
    features.push_back(*std::max_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end()));
    features.push_back(getMean(volume_WLH_ratios.begin(), volume_WLH_ratios.end(), noItems));
    features.push_back(getStd(volume_WLH_ratios.begin(), volume_WLH_ratios.end(), noItems));

    // Resize or pad to match model input if needed
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    return torch::from_blob(features.data(), {1, static_cast<int64_t>(features.size())}, options).clone();
}

float MLModelsContainer::classify(const std::vector<Cuboid>& items,
                                  const Collections::IdVector& route,
                                  const Container& container) {

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor output = model.forward({input}).toTensor();
    return output.item<float>();
}

}  // namespace Classifier
}  // namespace ContainerLoading
