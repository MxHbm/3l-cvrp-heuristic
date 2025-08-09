#pragma once

#include "Algorithms/LoadingStatus.h"

#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace VehicleRouting{
namespace Improvement{

    enum class ImprovementTypes
    {
        Intra,
        Inter,
        Perturbation
    };
}
}

namespace ContainerLoading
{

struct EnumClassHash {
    template<class T>
    std::size_t operator()(T v) const noexcept {
        return static_cast<std::size_t>(v);
    }
};

struct ContainerLoadingParams
{
    enum class VariantType
    {
        None,
        AllConstraints,
        NoFragility,
        NoSupport,
        NoLifo,
        LoadingOnly,
        VolumeWeightApproximation,
        Volume,
        Weight
    };

    VariantType Variant = VariantType::None;
    Algorithms::LoadingFlag LoadingFlags = Algorithms::LoadingFlag::NoneSet;
    bool EnableThreeDimensionalLoading = false;
    double SupportArea = 0.0;
    bool EnableSupport = false;
    bool EnableLifo = false;
    bool EnableFragility = false;
    
    //ClassifierParams
    std::string TracedModelPath{};
    std::string SerializeJson_MeanStd{};
    bool SaveTensorData = false;
    std::string TensorDataFilePath{};
    float AcceptanceThreshold{0.5f};

    //CPSolverParams
    int Threads = 8;
    int Seed = 0;
    bool LogFlag = true;
    bool Presolve = true;
    bool EnableCumulativeDimensions = false;
    bool EnableNoOverlap2DFloor = false;

    using IT = VehicleRouting::Improvement::ImprovementTypes;
    std::unordered_map<IT, bool, EnumClassHash> UseClassifierLocalSearch{
        {IT::Intra, true},
        {IT::Perturbation, true},
        {IT::Inter, false}
    };
    bool UseFilterStartSolution = false;
    
    [[nodiscard]] std::string GetVariantString() const {
        switch (Variant) {
            case VariantType::AllConstraints:            return "AllConstraints";
            case VariantType::LoadingOnly:               return "LoadingOnly";
            case VariantType::NoFragility:               return "NoFragility";
            case VariantType::NoLifo:                    return "NoLifo";
            case VariantType::NoSupport:                 return "NoSupport";
            case VariantType::Volume:                    return "Volume";
            case VariantType::VolumeWeightApproximation: return "VolumeWeightApproximation";
            case VariantType::Weight:                    return "Weight";
            default: {
                const auto v = static_cast<int>(Variant);
                throw std::runtime_error(("Variant " + std::to_string(v) + " is an invalid problem variant.").c_str());
            }
        }
    }

    void SetFlags()
    {
        switch (Variant)
        {
            case VariantType::AllConstraints:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.75;
                EnableSupport = true;
                EnableLifo = true;
                EnableFragility = true;
                LoadingFlags = Algorithms::LoadingFlag::Complete;
                break;
            }
            case VariantType::NoFragility:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.75;
                EnableSupport = true;
                EnableLifo = true;
                EnableFragility = false;
                LoadingFlags = Algorithms::LoadingFlag::NoFragility;
                break;
            }
            case VariantType::NoSupport:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = true;
                EnableFragility = true;
                LoadingFlags = Algorithms::LoadingFlag::NoSupport;
                break;
            }
            case VariantType::NoLifo:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.75;
                EnableSupport = true;
                EnableLifo = false;
                EnableFragility = true;
                LoadingFlags = Algorithms::LoadingFlag::NoLifo;
                break;
            }
            case VariantType::LoadingOnly:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = false;
                EnableFragility = false;
                LoadingFlags = Algorithms::LoadingFlag::LoadingOnly;
                break;
            }
            case VariantType::VolumeWeightApproximation:
            {
                EnableThreeDimensionalLoading = false;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = false;
                EnableFragility = false;
                break;
            }
            case VariantType::Volume:
            {
                EnableThreeDimensionalLoading = false;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = false;
                EnableFragility = false;
                break;
            }
            case VariantType::Weight:
            {
                EnableThreeDimensionalLoading = false;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = false;
                EnableFragility = false;
                break;
            }
            default:
                throw std::runtime_error("Problem variant not implemented.");
        }
    }

};
}
