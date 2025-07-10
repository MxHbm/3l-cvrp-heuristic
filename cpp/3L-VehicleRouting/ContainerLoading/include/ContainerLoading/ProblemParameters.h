#pragma once

#include "Algorithms/CPSolverParameters.h"
#include "Algorithms/LoadingStatus.h"
#include "Algorithms/ClassifierParameters.h"

#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace ContainerLoading
{
using namespace Algorithms;

struct LoadingProblemParams
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
    LoadingFlag LoadingFlags = LoadingFlag::NoneSet;
    bool EnableThreeDimensionalLoading = false;
    double SupportArea = 0.0;
    bool EnableSupport = false;
    bool EnableLifo = false;
    bool EnableFragility = false;

   [[nodiscard]] std::string GetVariantString() const
    {
        switch (Variant)
        {
            case VariantType::AllConstraints:
                return "AllConstraints";
            case VariantType::LoadingOnly:
                return "LoadingOnly";
            case VariantType::NoFragility:
                return "NoFragility";
            case VariantType::NoLifo:
                return "NoLifo";
            case VariantType::NoSupport:
                return "NoSupport";
            case VariantType::Volume:
                return "Volume";
            case VariantType::VolumeWeightApproximation:
                return "VolumeWeightApproximation";
            case VariantType::Weight:
                return "Weight";
            default:
                std::string message = "Variant " + std::to_string((int)Variant) + " is an invalid problem variant.";
                throw std::runtime_error(message.c_str());
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
                LoadingFlags = LoadingFlag::Complete;
                break;
            }
            case VariantType::NoFragility:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.75;
                EnableSupport = true;
                EnableLifo = true;
                EnableFragility = false;
                LoadingFlags = LoadingFlag::NoFragility;
                break;
            }
            case VariantType::NoSupport:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = true;
                EnableFragility = true;
                LoadingFlags = LoadingFlag::NoSupport;
                break;
            }
            case VariantType::NoLifo:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.75;
                EnableSupport = true;
                EnableLifo = false;
                EnableFragility = true;
                LoadingFlags = LoadingFlag::NoLifo;
                break;
            }
            case VariantType::LoadingOnly:
            {
                EnableThreeDimensionalLoading = true;
                SupportArea = 0.0;
                EnableSupport = false;
                EnableLifo = false;
                EnableFragility = false;
                LoadingFlags = LoadingFlag::LoadingOnly;
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
        };
    };
};

struct ContainerLoadingParams
{
    CPSolverParams CPSolver;
    LoadingProblemParams LoadingProblem;
    ClassifierParams classifierParams;
};

}
