#include "Helper/Serialization.h"

#include "ContainerLoading/Model/Container.h"

#include "Algorithms/BCRoutingParams.h"
#include "Model/Solution.h"

namespace VehicleRouting
{
namespace Algorithms
{

NLOHMANN_JSON_SERIALIZE_ENUM(IteratedLocalSearchParams::StartSolutionType,
                             {{IteratedLocalSearchParams::StartSolutionType::None, "None"},
                              {IteratedLocalSearchParams::StartSolutionType::ModifiedSavings, "ModifiedSavings"},
                              {IteratedLocalSearchParams::StartSolutionType::Savings, "Savings"},
                              {IteratedLocalSearchParams::StartSolutionType::SPHeuristic, "SPHeuristic"}});


NLOHMANN_JSON_SERIALIZE_ENUM(IteratedLocalSearchParams::CallType,
    {{IteratedLocalSearchParams::CallType::None, "None"},
    {IteratedLocalSearchParams::CallType::Exact, "Exact"},
    {IteratedLocalSearchParams::CallType::ExactLimit, "ExactLimit"},
    {IteratedLocalSearchParams::CallType::ILS, "ILS"},
    {IteratedLocalSearchParams::CallType::Constructive, "Constructive"}});

NLOHMANN_JSON_SERIALIZE_ENUM(LocalSearchTypes,
    {{LocalSearchTypes::None, "None"},
    {LocalSearchTypes::TwoOpt, "TwoOpt"},
    {LocalSearchTypes::InterSwap, "InterSwap"},
    {LocalSearchTypes::IntraSwap, "IntraSwap"},
    {LocalSearchTypes::InterInsertion, "InterInsertion"},
    {LocalSearchTypes::IntraInsertion, "IntraInsertion"},
    {LocalSearchTypes::DeleteEmptyRoutes, "DeleteEmptyRoutes"}
});

NLOHMANN_JSON_SERIALIZE_ENUM(PerturbationTypes,
    {{PerturbationTypes::None, "None"},
    {PerturbationTypes::K_RandomSwaps, "K_RandomSwaps"},
    {PerturbationTypes::K_RandomInsertions, "K_RandomInsertions"}});
}
}

namespace ContainerLoading
{
NLOHMANN_JSON_SERIALIZE_ENUM(LoadingProblemParams::VariantType,
                             {{LoadingProblemParams::VariantType::None, "None"},
                              {LoadingProblemParams::VariantType::AllConstraints, "AllConstraints"},
                              {LoadingProblemParams::VariantType::NoFragility, "NoFragility"},
                              {LoadingProblemParams::VariantType::NoSupport, "NoSupport"},
                              {LoadingProblemParams::VariantType::NoLifo, "NoLifo"},
                              {LoadingProblemParams::VariantType::LoadingOnly, "LoadingOnly"},
                              {LoadingProblemParams::VariantType::VolumeWeightApproximation,
                               "VolumeWeightApproximation"},
                              {LoadingProblemParams::VariantType::Volume, "Volume"},
                              {LoadingProblemParams::VariantType::Weight, "Weight"}});
}
namespace ContainerLoading
{
namespace Algorithms
{

void from_json(const json& j, CPSolverParams& params)
{
    j.at("EnableCumulativeDimensions").get_to(params.EnableCumulativeDimensions);
    j.at("EnableNoOverlap2DFloor").get_to(params.EnableNoOverlap2DFloor);
    j.at("LogFlag").get_to(params.LogFlag);
    j.at("Threads").get_to(params.Threads);
    j.at("Seed").get_to(params.Seed);
}

void to_json(json& j, const CPSolverParams& params)
{
    j = json{{"EnableCumulativeDimensions", params.EnableCumulativeDimensions},
             {"EnableNoOverlap2DFloor", params.EnableNoOverlap2DFloor},
             {"LogFlag", params.LogFlag},
             {"Threads", params.Threads},
             {"Seed", params.Seed}};
}

void from_json(const json& j, ClassifierParams& params)
{
    j.at("TracedModelPath").get_to(params.TracedModelPath);
    j.at("SerializeJson_MeanStd").get_to(params.SerializeJson_MeanStd);
    j.at("UseClassifier").get_to(params.UseClassifier);
    j.at("SaveTensorData").get_to(params.SaveTensorData);
    j.at("TensorDataFilePath").get_to(params.TensorDataFilePath);
    j.at("AcceptanceThreshold").get_to(params.AcceptanceThreshold);
}

void to_json(json& j, const ClassifierParams& params)
{
    j = json{{"TracedModelPath", params.TracedModelPath},
             {"SerializeJson_MeanStd", params.SerializeJson_MeanStd},
             {"UseClassifier", params.UseClassifier},
             {"SaveTensorData", params.SaveTensorData},
             {"TensorDataFilePath", params.TensorDataFilePath},
             {"AcceptanceThreshold", params.AcceptanceThreshold}};
}

}
}

namespace ContainerLoading
{
void from_json(const json& j, LoadingProblemParams& params)
{
    j.at("ProblemVariant").get_to(params.Variant);
    j.at("SupportArea").get_to(params.SupportArea);
}

void to_json(json& j, const LoadingProblemParams& params)
{
    j = json{{"ProblemVariant", params.Variant}, {"SupportArea", params.SupportArea}};
}
}

namespace VehicleRouting
{
namespace Algorithms
{

void from_json(const json& j, IteratedLocalSearchParams& params)
{
    j.at("Run_ILS").get_to(params.RunILS);
    j.at("Run_LS").get_to(params.RunLS);
    j.at("LimitNoImpr").get_to(params.NoImprLimit);
    j.at("K_RandomMoves").get_to(params.K_RandomMoves);
    j.at("SetPartHeurThreshold").get_to(params.SetPartitioningHeuristicThreshold);
    j.at("StartSolution").get_to(params.StartSolution);
    j.at("ActivateSetPartHeur").get_to(params.ActivateSetPartitioningHeuristic);
    j.at("ActivateIntraRouteImprovement").get_to(params.ActivateIntraRouteImprovement);
    j.at("IntraRouteFullEnumThreshold").get_to(params.IntraRouteFullEnumThreshold);
    j.at("TimeLimit").get_to(params.TimeLimits);
    j.at("LocalSearchTypes").get_to(params.localSearchTypes);
    j.at("PerturbationTypes").get_to(params.perturbationTypes);
}

void to_json(json& j, const IteratedLocalSearchParams& params)
{
    j = json{{"Run_ILS", params.RunILS},
             {"Run_LS", params.RunLS},
             {"LimitNoImpr", params.NoImprLimit},
             {"K_RandomMoves", params.K_RandomMoves},
             {"LocalSearchTypes",params.localSearchTypes},
             {"PerturbationTypes",params.perturbationTypes},
             {"SetPartHeurThreshold", params.SetPartitioningHeuristicThreshold},
             {"StartSolution", params.StartSolution},
             {"ActivateSetPartHeur", params.ActivateSetPartitioningHeuristic},
             {"ActivateIntraRouteImprovement", params.ActivateIntraRouteImprovement},
             {"IntraRouteFullEnumThreshold", params.IntraRouteFullEnumThreshold},
             {"TimeLimit", params.TimeLimits}};
}



void from_json(const json& j, InputParameters& inputParameters)
{
    j.at("LoadingProblemParams").get_to(inputParameters.ContainerLoading.LoadingProblem);
    j.at("IteratedLocalSearchParams").get_to(inputParameters.IteratedLocalSearch);
    j.at("CPSolverParams").get_to(inputParameters.ContainerLoading.CPSolver);
    j.at("ClassifierParams").get_to(inputParameters.ContainerLoading.classifierParams);
}

void to_json(json& j, const InputParameters& inputParameters)
{
    j = json{{"LoadingProblemParams", inputParameters.ContainerLoading.LoadingProblem},
             {"IteratedLocalSearchParams", inputParameters.IteratedLocalSearch},
             {"CPSolverParams", inputParameters.ContainerLoading.CPSolver},
             {"ClassifierParams",inputParameters.ContainerLoading.classifierParams}};
}

}
}

namespace VehicleRouting
{
namespace Helper
{
void from_json(const json& j, Helper::Timer& timer) {}

void to_json(json& j, const Helper::Timer& timer)
{
    j = json{
        {"InfeasibleArcs", timer.InfeasibleArcs.count()},
        {"LowerBoundVehicles", timer.LowerBoundVehicles.count()},
        {"StartSolution", timer.StartSolution.count()},
        {"MetaHeuristic", timer.MetaHeuristic.count()},
    };
}
}
}

namespace VehicleRouting
{
namespace Model
{
using namespace ContainerLoading::Model;

void from_json(const json& j, Node& node)
{
    j.at("InternId").get_to(node.InternId);
    j.at("Latitude").get_to(node.PositionY);
    j.at("Longitude").get_to(node.PositionX);
    j.at("TotalWeight").get_to(node.TotalWeight);
    j.at("Items").get_to<std::vector<Cuboid>>(node.Items);
}

void to_json(json& j, const Node& node)
{
    j = json{
        {"InternId", node.InternId},
        {"Latitude", node.PositionY},
        {"Longitude", node.PositionX},
        {"TotalWeight", node.TotalWeight},
        {"Items", node.Items},
    };
}

void from_json(const json& j, Vehicle& vehicle)
{
    j.at("InternId").get_to(vehicle.InternId);
    j.at("Volume").get_to(vehicle.Volume);
    j.at("Containers").get_to<std::vector<Container>>(vehicle.Containers);
}

void to_json(json& j, const Vehicle& vehicle)
{
    j = json{
        {"InternId", vehicle.InternId},
        {"Volume", vehicle.Volume},
        {"Containers", vehicle.Containers},
    };
}

void from_json(const json& j, Tour& tour)
{
    j.at("Depot").get_to<Node>(tour.Depot);
    j.at("Vehicle").get_to<Vehicle>(tour.Vehicle);
    j.at("Route").get_to<std::vector<Node>>(tour.Route);
}

void to_json(json& j, const Tour& tour)
{
    j = json{
        {"Depot", tour.Depot},
        {"Vehicle", tour.Vehicle},
        {"Route", tour.Route},
    };
}

void from_json(const json& j, OutputSolution& solution) { j.at("Tours").get_to<std::vector<Tour>>(solution.Tours); }

void to_json(json& j, const OutputSolution& solution)
{
    j = json{
        {"Tours", solution.Tours},
        {"Costs", solution.Costs},
        {"NumberRoutes", solution.NumberOfRoutes},
        {"LowerBoundVehicles", solution.LowerBoundVehicles},
    };
}

//void from_json(const json& j, SolverStatistics& statistics) { j.at("Runtime").get_to(statistics.Runtime); }

void to_json(json& j, const SolverStatistics& statistics)
{
    j = json{
        {"ILSIterations", statistics.ILSIterationCount},
        {"CP_Rejections", statistics.rejectionCount},
        {"DeletedArcs", statistics.DeletedArcs},
        {"InfTailPath", statistics.InfeasibleTailPathStart},
        {"Timer", statistics.Timer},
        {"SolutionProgress", statistics.solutionTracker}
    };
}

void from_json(const json& j, SolutionFile& solution)
{
    j.at("InputParameters").get_to<InputParameters>(solution.InputParameters);
    ////j.at("SolverStatistics").get_to<SolverStatistics>(solution.SolverStatistics);
    j.at("Solution").get_to<OutputSolution>(solution.OutputSolution);
}

void to_json(json& j, const SolutionFile& solution)
{
    j = json{
        {"InputParameters", solution.InputParameters},
        ////{"SolverStatistics", solution.SolverStatistics},
        {"Solution", solution.OutputSolution},
    };
}

void from_json(const json& j, SolutionTracker& tracker) {}

void to_json(json& j, const SolutionTracker& tracker)
{
    j = json{{"CurrSolProgress", tracker.CurrentSolution},
             {"BestSolProgress", tracker.BestSolution}};
}


}
}