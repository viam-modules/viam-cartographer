// This is an experimental integration of cartographer into RDK.
#include "cartographer/mapping/map_builder.h"

#include <sstream>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "glog/logging.h"
#include "io.h"
# include "map_builder.h"

namespace viam {
namespace carto_facade {
MapBuilder::~MapBuilder() {
    // This is needed as the google cartographer MapBuilder class
    // doesn't have a destructor which cleans up all the TrajectoryBuilder
    // instances started from a given MapBuilder.
    // This destructor ensures the TrajectoryBuilder is cleaned up with
    // the MapBuilder.
    //
    // If we don't do this, there will still be trajectory related
    // threads doing work in google cartographer, which will try
    // to access the deleted MapBuilder, which will throw exceptions.
    if (map_builder_ != nullptr && trajectory_builder != nullptr) {
        map_builder_->FinishTrajectory(trajectory_id);
    }
}

std::vector<::cartographer::transform::Rigid3d>
MapBuilder::GetLocalSlamResultPoses() {
    return local_slam_result_poses_;
}

void MapBuilder::SetUp(std::string configuration_directory,
                       std::string configuration_basename) {
    VLOG(1) << "MapBuilder::SetUp configuration_directory: "
            << configuration_directory
            << " configuration_basename: " << configuration_basename;
    auto file_resolver =
        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string lua_code =
        file_resolver->GetFileContentOrDie(configuration_basename);

    auto options =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(
            lua_code, std::move(file_resolver));

    auto map_builder_parameters = options->GetDictionary("map_builder");
    auto trajectory_builder_parameters =
        options->GetDictionary("trajectory_builder");

    map_builder_options_ = cartographer::mapping::CreateMapBuilderOptions(
        map_builder_parameters.get());
    trajectory_builder_options_ =
        cartographer::mapping::CreateTrajectoryBuilderOptions(
            trajectory_builder_parameters.get());
}

void MapBuilder::BuildMapBuilder() {
    VLOG(1) << "MapBuilder::BuildMapBuilder";
    map_builder_ =
        cartographer::mapping::CreateMapBuilder(map_builder_options_);
}

void MapBuilder::LoadMapFromFile(std::string internal_state_filename,
                                 bool load_frozen_trajectory,
                                 bool optimize_on_start) {
    VLOG(1) << "calling map_builder.LoadMapFromFile "
               "latest_internal_state_filename: "
            << internal_state_filename
            << " load_frozen_trajectory: " << load_frozen_trajectory
            << " algo_config.optimize_on_start: " << optimize_on_start;
    data_start_time = viam::carto_facade::io::ReadTimeFromTimestamp(
        internal_state_filename.substr(
            internal_state_filename.find(
                viam::carto_facade::io::filename_prefix) +
                viam::carto_facade::io::filename_prefix.length(),
            internal_state_filename.find(".pbstream")));

    std::map<int, int> trajectory_ids_map = map_builder_->LoadStateFromFile(
        internal_state_filename, load_frozen_trajectory);

    if (optimize_on_start) {
        LOG(INFO) << "Optimizing map on start, this may take a few minutes";
        map_builder_->pose_graph()->RunFinalOptimization();
    }
    for (auto &&trajectory_ids_pair : trajectory_ids_map)
        VLOG(1) << "Trajectory ids mapping from apriori map: "
                << trajectory_ids_pair.first << " "
                << trajectory_ids_pair.second;
}

bool MapBuilder::SaveMapToFile(bool include_unfinished_submaps,
                               const std::string filename_with_timestamp) {
    bool ok = map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                                 filename_with_timestamp);
    if (!ok) {
        LOG(ERROR) << "Saving the map to pbstream failed.";
    }
    return ok;
}

void MapBuilder::AddSensorData(
    cartographer::sensor::TimedPointCloudData measurement) {
    VLOG(1) << "AddSensorData: measurement.time: " << measurement.time
            << "cartographer::common::ToUniversal: "
            << cartographer::common::ToUniversal(measurement.time);
    VLOG(1) << measurement.ranges[0].position[0] << " "
            << measurement.ranges[0].position[1] << " "
            << measurement.ranges[0].position[2];
    /* for (size_t i = 0; i < measurement.ranges.size(); i++) { */
    /*   VLOG(1) << measurement.ranges[i].position[0] << " " <<
     * measurement.ranges[i].position[1] << " " <<
     * measurement.ranges[i].position[2]; */
    /* } */
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
}

void MapBuilder::StartLidarTrajectoryBuilder() {
    VLOG(1) << "MapBuilder::StartLidarTrajectoryBuilder";
    trajectory_id = map_builder_->AddTrajectoryBuilder(
        {kRangeSensorId}, trajectory_builder_options_,
        GetLocalSlamResultCallback());
    VLOG(1) << "Using trajectory ID: " << trajectory_id;

    trajectory_builder = map_builder_->GetTrajectoryBuilder(trajectory_id);
}

cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback
MapBuilder::GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
        local_slam_result_poses_.push_back(local_pose);
    };
}

void MapBuilder::SetStartTime(double input_start_time) {
    start_time = input_start_time;
}

cartographer::sensor::TimedPointCloudData MapBuilder::GetDataFromFile(
    std::string file) {
    cartographer::sensor::TimedPointCloudData point_cloud;

    if (start_time == -1) {
        throw std::runtime_error("start_time has not been initialized");
    }
    point_cloud = viam::carto_facade::io::TimedPointCloudDataFromPCDBuilder(
        file, start_time);

    return point_cloud;
}

// TODO: There might still be a lot of room to improve accuracy & speed.
// Might be worth investigating in the future.
cartographer::transform::Rigid3d MapBuilder::GetGlobalPose(
    cartographer::transform::Rigid3d &local_pose) {
    auto local_transform =
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
    return local_transform * local_pose;
}

void MapBuilder::OverwriteOptimizeEveryNNodes(int value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    mutable_pose_graph_options->set_optimize_every_n_nodes(value);
}

void MapBuilder::OverwriteNumRangeData(int value) {
    auto mutable_trajectory_builder_2d_options =
        trajectory_builder_options_.mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->mutable_submaps_options()
        ->set_num_range_data(value);
}

void MapBuilder::OverwriteMissingDataRayLength(float value) {
    auto mutable_trajectory_builder_2d_options =
        trajectory_builder_options_.mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->set_missing_data_ray_length(value);
}

void MapBuilder::OverwriteMaxRange(float value) {
    auto mutable_trajectory_builder_2d_options =
        trajectory_builder_options_.mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->set_max_range(value);
}

void MapBuilder::OverwriteMinRange(float value) {
    auto mutable_trajectory_builder_2d_options =
        trajectory_builder_options_.mutable_trajectory_builder_2d_options();
    mutable_trajectory_builder_2d_options->set_min_range(value);
}

void MapBuilder::OverwriteMaxSubmapsToKeep(int value) {
    trajectory_builder_options_.mutable_pure_localization_trimmer()
        ->set_max_submaps_to_keep(value);
}

void MapBuilder::OverwriteFreshSubmapsCount(int value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    auto mutable_overlapping_submaps_trimmer_2d =
        mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    mutable_overlapping_submaps_trimmer_2d->set_fresh_submaps_count(value);
}

void MapBuilder::OverwriteMinCoveredArea(double value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    auto mutable_overlapping_submaps_trimmer_2d =
        mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    mutable_overlapping_submaps_trimmer_2d->set_min_covered_area(value);
}

void MapBuilder::OverwriteMinAddedSubmapsCount(int value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    auto mutable_overlapping_submaps_trimmer_2d =
        mutable_pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    mutable_overlapping_submaps_trimmer_2d->set_min_added_submaps_count(value);
}

void MapBuilder::OverwriteOccupiedSpaceWeight(double value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    auto mutable_ceres_scan_matcher_options =
        mutable_pose_graph_options->mutable_constraint_builder_options()
            ->mutable_ceres_scan_matcher_options();

    mutable_ceres_scan_matcher_options->set_occupied_space_weight(value);
}

void MapBuilder::OverwriteTranslationWeight(double value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    auto mutable_ceres_scan_matcher_options =
        mutable_pose_graph_options->mutable_constraint_builder_options()
            ->mutable_ceres_scan_matcher_options();

    mutable_ceres_scan_matcher_options->set_translation_weight(value);
}

void MapBuilder::OverwriteRotationWeight(double value) {
    auto mutable_pose_graph_options =
        map_builder_options_.mutable_pose_graph_options();
    auto mutable_ceres_scan_matcher_options =
        mutable_pose_graph_options->mutable_constraint_builder_options()
            ->mutable_ceres_scan_matcher_options();

    mutable_ceres_scan_matcher_options->set_rotation_weight(value);
}

int MapBuilder::GetOptimizeEveryNNodes() {
    return map_builder_options_.pose_graph_options().optimize_every_n_nodes();
}

int MapBuilder::GetNumRangeData() {
    return trajectory_builder_options_.trajectory_builder_2d_options()
        .submaps_options()
        .num_range_data();
}

float MapBuilder::GetMissingDataRayLength() {
    return trajectory_builder_options_.trajectory_builder_2d_options()
        .missing_data_ray_length();
}

float MapBuilder::GetMaxRange() {
    return trajectory_builder_options_.trajectory_builder_2d_options()
        .max_range();
}

float MapBuilder::GetMinRange() {
    return trajectory_builder_options_.trajectory_builder_2d_options()
        .min_range();
}

int MapBuilder::GetMaxSubmapsToKeep() {
    return trajectory_builder_options_.pure_localization_trimmer()
        .max_submaps_to_keep();
}

int MapBuilder::GetFreshSubmapsCount() {
    return map_builder_options_.pose_graph_options()
        .overlapping_submaps_trimmer_2d()
        .fresh_submaps_count();
}

double MapBuilder::GetMinCoveredArea() {
    return map_builder_options_.pose_graph_options()
        .overlapping_submaps_trimmer_2d()
        .min_covered_area();
}

int MapBuilder::GetMinAddedSubmapsCount() {
    return map_builder_options_.pose_graph_options()
        .overlapping_submaps_trimmer_2d()
        .min_added_submaps_count();
}

double MapBuilder::GetOccupiedSpaceWeight() {
    return map_builder_options_.pose_graph_options()
        .constraint_builder_options()
        .ceres_scan_matcher_options()
        .occupied_space_weight();
}

double MapBuilder::GetTranslationWeight() {
    return map_builder_options_.pose_graph_options()
        .constraint_builder_options()
        .ceres_scan_matcher_options()
        .translation_weight();
}

double MapBuilder::GetRotationWeight() {
    return map_builder_options_.pose_graph_options()
        .constraint_builder_options()
        .ceres_scan_matcher_options()
        .rotation_weight();
}

}  // namespace carto_facade
}  // namespace viam
