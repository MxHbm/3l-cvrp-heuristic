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
};

}
}