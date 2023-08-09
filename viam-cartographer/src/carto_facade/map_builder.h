// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTO_FACADE_MAP_BUILDER_H
#define VIAM_CARTO_FACADE_MAP_BUILDER_H

#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace viam {
namespace carto_facade {
using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};

class MapBuilder {
   public:
    ~MapBuilder();
    // SetUp reads in the cartographer parameters via reading in the lua files.
    void SetUp(std::string configuration_directory,
               std::string configuration_basename);

    // BuildMapBuilder creates the internal map_builder_ using the read in
    // cartographer parameters.
    void BuildMapBuilder();

    // LoadMapFromFile sets the state of the internal map_builder_ based on
    // the provided apriori map. It does also set cartographer to either run
    // in updating or localizing mode, depending on the load_frozen_trajectory
    // value.
    void LoadMapFromFile(std::string map_filename, bool load_frozen_trajectory,
                         bool optimize_on_start);

    // SaveMapToFile saves the current map_builder_ state to a pbstream file at
    // the provided path.
    bool SaveMapToFile(bool include_unfinished_submaps,
                       const std::string filename_with_timestamp);

    // SaveMapToStream converted the saved pbstream to a stream and deletes the
    // file.
    std::string ConvertSavedMapToStream(
        const std::string filename_with_timestamp, std::string *buffer);

    // TryFileClose attempts to close an opened ifstream, returning an error
    // string if it fails.
    std::string TryFileClose(std::ifstream &file, std::string filename);

    void StartLidarTrajectoryBuilder();

    // SetStartTime sets the start_time to the time stamp from the first sensor
    // file that is being read in.
    void SetStartTime(double input_start_time);

    // GetDataFromFile creates a TimedPointCloudData object from reading in
    // a PCD file.
    cartographer::sensor::TimedPointCloudData GetDataFromFile(std::string file);

    // GetGlobalPose returns the local pose based on the provided a local pose.
    cartographer::transform::Rigid3d GetGlobalPose();

    // AddSensorData adds sensor data to cartographer's internal state
    // throws if adding sensor data fails.
    void AddSensorData(const std::string &sensor_id,
                       cartographer::sensor::TimedPointCloudData measurement);
    void AddSensorData(const std::string &sensor_id,
                       cartographer::sensor::ImuData measurement);

    // GetLocalSlamResultCallback saves the local pose in the
    // local_slam_result_poses array.
    cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback
    GetLocalSlamResultCallback();

    // Overwrite functions to overwrite the exposed cartographer parameters.
    void OverwriteOptimizeEveryNNodes(int value);
    void OverwriteNumRangeData(int value);
    void OverwriteMissingDataRayLength(float value);
    void OverwriteMaxRange(float value);
    void OverwriteMinRange(float value);
    void OverwriteMaxSubmapsToKeep(int value);
    void OverwriteFreshSubmapsCount(int value);
    void OverwriteMinCoveredArea(double value);
    void OverwriteMinAddedSubmapsCount(int value);
    void OverwriteOccupiedSpaceWeight(double value);
    void OverwriteTranslationWeight(double value);
    void OverwriteRotationWeight(double value);

    // Getter functions to return the exposed cartographer parameters.
    int GetOptimizeEveryNNodes();
    int GetNumRangeData();
    float GetMissingDataRayLength();
    float GetMaxRange();
    float GetMinRange();
    int GetMaxSubmapsToKeep();
    int GetFreshSubmapsCount();
    double GetMinCoveredArea();
    int GetMinAddedSubmapsCount();
    double GetOccupiedSpaceWeight();
    double GetTranslationWeight();
    double GetRotationWeight();

    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder;
    int trajectory_id;
    cartographer::mapping::proto::MapBuilderOptions map_builder_options_;
    cartographer::mapping::proto::TrajectoryBuilderOptions
        trajectory_builder_options_;

   private:
    std::mutex local_slam_result_pose_mutex;
    ::cartographer::transform::Rigid3d local_slam_result_pose =
        cartographer::transform::Rigid3d();
    ;
    double start_time = -1;
};
}  // namespace carto_facade
}  // namespace viam

#endif  // VIAM_CARTO_FACADE_MAP_BUILDER_H
