#pragma once

#include <string>

namespace ContainerLoading
{
namespace Algorithms
{
struct ClassifierParams
{
    std::string TracedModelPath = "";
    std::string SerializeJson_MeanStd = "";
    bool UseClassifier = false;
    bool SaveTensorData = false;
    std::string TensorDataFilePath = "";
    float AcceptanceThreshold{0.5f};
};

}
}