// File: Classifier.cpp

#include "LoadingChecker/Classifier.h"
#include <cmath>
#include <fstream>
#include "nlohmann/json.hpp"
#include <filesystem>

namespace ContainerLoading {

void Classifier::loadStandardScalingFromJson(const std::string& scaler_path){

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

Classifier::Classifier(const ClassifierParams& classifierParams) : mClassifierParams(classifierParams)
{

    model = torch::jit::load(classifierParams.TracedModelPath);
    model.eval();

    loadStandardScalingFromJson(classifierParams.SerializeJson_MeanStd);

}

torch::Tensor Classifier::applyStandardScaling(const torch::Tensor& input) const{

    return (input - mean_tensor) / std_tensor;
}

float Classifier::getMean(std::vector<float>::iterator first,
                          std::vector<float>::iterator last)
{
    auto count = std::distance(first, last);
    if (count == 0) return 0.0f;

    float sum = std::accumulate(first, last, 0.0f);
    return sum / count;
}


float Classifier::getStd(std::vector<float>::iterator first,
                         std::vector<float>::iterator last)
{
    auto count = std::distance(first, last);
    if (count <= 1) return 0.0f;

    float mean = getMean(first, last);

    float variance = 0.0f;
    for (auto it = first; it != last; ++it)
        variance += std::pow(*it - mean, 2);

    return std::sqrt(variance / count);  // or / (count - 1) for sample stddev
}




//['NoItems', 'NoCustomers', 'Rel Volume', 'Rel Weight', 'Weight Distribution', 'Volume Distribution', 'Fragile Ratio',
//'Rel Total Length Items', 'Rel Total Width Items', 'Rel Total Height Items']

torch::Tensor Classifier::extractFeatures(const std::vector<Cuboid>& items,
                                          const Collections::IdVector& route,
                                          const Container& container) const {

    torch::Tensor result = torch::zeros({1,46});
    //std::vector<float> features;
    //features.reserve(38);
    
    const auto containerWeightLimit = container.WeightLimit;
    const float containerVolume = container.Volume;
    const float noItems = items.size();
    const float noCustomers = route.size();
    const float containerDx = container.Dx;
    const float containerDy = container.Dy;
    const float containerDz = container.Dz;
    const float containerArea = container.Area;

    std::vector<int> pyramideValues(route.size());
    std::iota(pyramideValues.begin(), pyramideValues.end(), 1);


	std::vector<float> width_height_ratios(noItems, 0.0f);
    std::vector<float> length_height_ratios(noItems, 0.0f);
    std::vector<float> width_length_ratios(noItems, 0.0f);
    std::vector<float> length_L_ratios(noItems, 0.0f);
    std::vector<float> width_W_ratios(noItems, 0.0f);
    std::vector<float> height_H_ratios(noItems, 0.0f);
    std::vector<float> volume_WLH_ratios(noItems, 0.0f);
    std::vector<float> height_area_ratios(noItems, 0.0f);
    std::vector<float> area_AREA_ratios(noItems, 0.0f);

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
        tot_width += item.Dy;
        tot_length += item.Dx;
        tot_height += item.Dz;
        if(item.Fragility == Fragility::Fragile){
            ++fragile_count;
        }
        //Distributions
        volumeDistribution += item.Volume * pyramideValues[item.GroupId];
        weightDistribution += item.Weight * pyramideValues[item.GroupId];

        // Add more as needed, matching the Python training input

        width_height_ratios[it] = static_cast<float>(item.Dy) / item.Dz;
        length_height_ratios[it] = static_cast<float>(item.Dx) / item.Dz;
        width_length_ratios[it] = static_cast<float>(item.Dy) / item.Dx;
        length_L_ratios[it] = item.Dx / containerDx;
        width_W_ratios[it] = item.Dy / containerDy;
        height_H_ratios[it] = item.Dz / containerDz;
        volume_WLH_ratios[it] = item.Volume / containerVolume;
        height_area_ratios[it] = item.Dz / item.Area;
        area_AREA_ratios[it] = item.Area / containerArea;
        ++it;
    }
    //NoItems 
    result[0][0] = noItems;
    //No Customers
    result[0][1] = noCustomers;

    //'Rel Volume' and 'Rel Weight'
    result[0][2] = tot_volume / containerVolume;
    result[0][3] = tot_weight / containerWeightLimit;

    //'Weight Distribution', 'Volume Distribution'
    result[0][4] = weightDistribution / containerWeightLimit;
    result[0][5] = volumeDistribution / containerVolume;

    //'Fragile Ratio'
    result[0][6] = fragile_count / noItems;

    //Rel Total Length Items', 'Rel Total Width Items', 'Rel Total Height Items', 
    result[0][7] = tot_length / containerDx;
    result[0][8] = tot_width / containerDy;
    result[0][9] = tot_height / containerDz;

    // 'width_height_min', 'width_height_max', 'width_height_mean', 'width_height_std',
    result[0][10] = *std::min_element(width_height_ratios.begin(), width_height_ratios.end());
    result[0][11] = *std::max_element(width_height_ratios.begin(), width_height_ratios.end());
    result[0][12] = getMean(width_height_ratios.begin(), width_height_ratios.end());
    result[0][13] = getStd(width_height_ratios.begin(), width_height_ratios.end());
    
    //'length_height_min', 'length_height_max', 'length_height_mean', 'length_height_std',
    result[0][14] = *std::min_element(length_height_ratios.begin(), length_height_ratios.end());
    result[0][15] = *std::max_element(length_height_ratios.begin(), length_height_ratios.end());
    result[0][16] = getMean(length_height_ratios.begin(), length_height_ratios.end());
    result[0][17] = getStd(length_height_ratios.begin(), length_height_ratios.end());

    // 'width_length_min', 'width_length_max', 'width_length_mean', 'width_length_std',
    result[0][18] = *std::min_element(width_length_ratios.begin(), width_length_ratios.end());
    result[0][19] = *std::max_element(width_length_ratios.begin(), width_length_ratios.end());
    result[0][20] = getMean(width_length_ratios.begin(), width_length_ratios.end());
    result[0][21] = getStd(width_length_ratios.begin(), width_length_ratios.end());

    // 'width_W_min', 'width_W_max', 'width_W_mean', 'width_W_std', 
    result[0][22] = *std::min_element(width_W_ratios.begin(), width_W_ratios.end());
    result[0][23] = *std::max_element(width_W_ratios.begin(), width_W_ratios.end());
    result[0][24] = getMean(width_W_ratios.begin(), width_W_ratios.end());
    result[0][25] = getStd(width_W_ratios.begin(), width_W_ratios.end());
    
    //'length_L_min', 'length_L_max', 'length_L_mean', 'length_L_std',
    result[0][26] = *std::min_element(length_L_ratios.begin(), length_L_ratios.end());
    result[0][27] = *std::max_element(length_L_ratios.begin(), length_L_ratios.end());
    result[0][28] = getMean(length_L_ratios.begin(), length_L_ratios.end());
    result[0][29] = getStd(length_L_ratios.begin(), length_L_ratios.end());

    //'height_H_min', 'height_H_max', 'height_H_mean', 'height_H_std'
    result[0][30] = *std::min_element(height_H_ratios.begin(), height_H_ratios.end());
    result[0][31] = *std::max_element(height_H_ratios.begin(), height_H_ratios.end());
    result[0][32] = getMean(height_H_ratios.begin(), height_H_ratios.end());
    result[0][33] = getStd(height_H_ratios.begin(), height_H_ratios.end());
    
    //'volume_WLH_min', 'volume_WLH_max', 'volume_WLH_mean', 'volume_WLH_std'
    result[0][34] = *std::min_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][35] = *std::max_element(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][36] = getMean(volume_WLH_ratios.begin(), volume_WLH_ratios.end());
    result[0][37] = getStd(volume_WLH_ratios.begin(), volume_WLH_ratios.end());


    result[0][38] = *std::min_element(height_area_ratios.begin(), height_area_ratios.end());
    result[0][39] = *std::max_element(height_area_ratios.begin(), height_area_ratios.end());
    result[0][40] = getMean(height_area_ratios.begin(), height_area_ratios.end());
    result[0][41] = getStd(height_area_ratios.begin(), height_area_ratios.end());


    result[0][42] = *std::min_element(area_AREA_ratios.begin(), area_AREA_ratios.end());
    result[0][43] = *std::max_element(area_AREA_ratios.begin(), area_AREA_ratios.end());
    result[0][44] = getMean(area_AREA_ratios.begin(), area_AREA_ratios.end());
    result[0][45] = getStd(area_AREA_ratios.begin(), area_AREA_ratios.end());

    // Resize or pad to match model input if needed
    return result;
}

std::string Classifier::get_timestamp() {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::tm* parts = std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(parts, "%Y-%m-%d_%H-%M-%S");
    oss << "-" << std::setw(3) << std::setfill('0') << now_ms.count();  // add milliseconds

    return oss.str();
}


// Save a 1D or 2D tensor as CSV with timestamp
void Classifier::save_tensor_to_csv(const torch::Tensor& tensor, const int status, const float output) {
    torch::Tensor cpu_tensor = tensor.detach().cpu();
    std::string filename = mClassifierParams.TensorDataFilePath + "/tensor_" + get_timestamp() + ".csv";

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n";
        return;
    }

    torch::Tensor tensor_2d = cpu_tensor;
    if (tensor_2d.dim() == 1) {
        tensor_2d = tensor_2d.unsqueeze(0);  // make it 2D: [1, N]
    }

    auto accessor = tensor_2d.accessor<float, 2>();
    for (int i = 0; i < tensor_2d.size(0); ++i) {
        // First column: status
        file << status << "," << output << ",";

        for (int j = 0; j < tensor_2d.size(1); ++j) {
            file << accessor[i][j];
            if (j < tensor_2d.size(1) - 1)
                file << ",";
        }
        file << "\n";
    }

    file.close();
}


void Classifier::saveClassifierResults(const std::vector<Cuboid>& items,
                                                const Collections::IdVector& route,
                                                const Container& container,
                                                const float output,
                                                const int status) {

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor input_scaled = applyStandardScaling(input);

    save_tensor_to_csv(input_scaled, status, output);
}

float Classifier::classifyReturnOutput(const std::vector<Cuboid>& items,
                            const Collections::IdVector& route,
                            const Container& container){

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor input_scaled = applyStandardScaling(input);
    torch::Tensor output = model.forward({input_scaled}).toTensor();

    return output.item<float>();
}

bool Classifier::classify(const std::vector<Cuboid>& items,
                           const Collections::IdVector& route,
                           const Container& container) {

    torch::Tensor input = extractFeatures(items, route, container);
    torch::Tensor input_scaled = applyStandardScaling(input);
    torch::Tensor output = model.forward({input_scaled}).toTensor();

    return output.item<float>() > mClassifierParams.AcceptanceThreshold;
}

}  // namespace ContainerLoading
