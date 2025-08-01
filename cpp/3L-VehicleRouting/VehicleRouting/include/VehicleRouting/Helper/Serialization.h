#pragma once

#include "ContainerLoading/Helper/Serialization.h"

#include "nlohmann/json.hpp"

using json = nlohmann::json;

#include <fstream>
#include <iomanip>

class Timer;

struct IteratedLocalSearchParams;
struct CPSolverParams;
struct LoadingProblemParams;
struct ClassifierParams;

class InputParameters;

namespace VehicleRouting
{
namespace Algorithms
{
void from_json(const json& j, IteratedLocalSearchParams& params);
void to_json(json& j, const IteratedLocalSearchParams& params);

void from_json(const json& j, InputParameters& inputParameters);
void to_json(json& j, const InputParameters& inputParameters);
}
}

namespace ContainerLoading
{
void from_json(const json& j, LoadingProblemParams& params);
void to_json(json& j, const LoadingProblemParams& params);
}

namespace ContainerLoading
{
namespace Algorithms
{
void from_json(const json& j, CPSolverParams& params);
void to_json(json& j, const CPSolverParams& params);

void from_json(const json& j, ClassifierParams& params);
void to_json(json& j, const ClassifierParams& params);

}
}

namespace VehicleRouting
{
namespace Model
{
// Note, to acces enums via json.at("EnumName").get<EnumType>(), they must be declared in the head file, which requires
// forward declaration. Because nested structs/enums cannot be forward declared (see
// https://stackoverflow.com/a/1021809) this is not possible for some enums.

class Vehicle;
class Node;
class SolverStatistics;
class Solution;
class SolutionFile;
class Tour;
class OutputSolution;
class SolutionTracker;

void from_json(const json& j, Node& node);
void to_json(json& j, const Node& node);

void from_json(const json& j, Vehicle& vehicle);
void to_json(json& j, const Vehicle& vehicle);

void from_json(const json& j, Tour& tour);
void to_json(json& j, const Tour& tour);

void from_json(const json& j, SolverStatistics& statistics);
void to_json(json& j, const SolverStatistics& statistics);

void from_json(const json& j, OutputSolution& solution);
void to_json(json& j, const OutputSolution& solution);

void from_json(const json& j, SolutionFile& solutionFile);
void to_json(json& j, const SolutionFile& solutionFile);

void from_json(const json& j, SolutionTracker& tracker);
void to_json(json& j, const SolutionTracker& tracker);
}
}

namespace VehicleRouting
{
namespace Helper
{
void from_json(const json& j, Timer& timer);
void to_json(json& j, const Timer& timer);
}
}

class Serializer
{
  public:
    template <class T>
    static void WriteToJson(const T& classToSerialize, std::string& folderPath, std::string& fileName)
    {
        std::ofstream ofs(folderPath + fileName + ".json");
        {
            if (!ofs.is_open())
            {
                throw std::runtime_error("File stream not open.");
            }

            json jsonFile = classToSerialize;
            ofs << std::setw(2) << jsonFile;
        }

        ofs.close();
    }
};