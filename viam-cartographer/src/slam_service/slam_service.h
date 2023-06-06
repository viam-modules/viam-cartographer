// This is an experimental integration of cartographer into RDK.
#ifndef SLAM_SERVICE_H_
#define SLAM_SERVICE_H_

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_context.h>

#include <atomic>
#include <shared_mutex>
#include <string>

#include "../io/draw_trajectories.h"
#include "../io/file_handler.h"
#include "../mapping/map_builder.h"
#include "../utils/slam_service_helpers.h"
#include "Eigen/Core"
#include "cairo/cairo.h"
#include "cartographer/io/submap_painter.h"
#include "common/v1/common.grpc.pb.h"
#include "common/v1/common.pb.h"
#include "service/slam/v1/slam.grpc.pb.h"
#include "service/slam/v1/slam.pb.h"
#include "src/carto_facade/bstrlib.h"
#include "src/carto_facade/carto_facade.h"

using google::protobuf::Struct;
using grpc::ServerContext;
using grpc::ServerWriter;
using viam::common::v1::Pose;
using viam::service::slam::v1::GetInternalStateRequest;
using viam::service::slam::v1::GetInternalStateResponse;
using viam::service::slam::v1::GetPointCloudMapRequest;
using viam::service::slam::v1::GetPointCloudMapResponse;
using viam::service::slam::v1::GetPositionRequest;
using viam::service::slam::v1::GetPositionResponse;
using viam::service::slam::v1::SLAMService;

namespace viam {

static const int checkForShutdownIntervalMicroseconds = 1e5;

// Byte limit on GRPC, used to help determine sampling skip_count
static const int maximumGRPCByteLimit = 32 * 1024 * 1024;
// Byte limit for chunks on GRPC, used for streaming apis
static const int maximumGRPCByteChunkSize = 1 * 1024 * 1024;
// The resolutionMeters variable defines the area in meters that each pixel
// represents. This is used to draw the cairo map and in so doing defines the
// resolution of the outputted PCD
static const double resolutionMeters = 0.05;

// Error log for when no submaps exist
static const std::string errorNoSubmaps = "No submaps to paint";

extern std::atomic<bool> b_continue_session;

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};

class SLAMServiceImpl final : public SLAMService::Service {
   public:
    // GetPosition returns the relative pose of the robot w.r.t the "origin"
    // of the map, which is the starting point from where the map was initially
    // created along with a component reference.
    ::grpc::Status GetPosition(ServerContext *context,
                               const GetPositionRequest *request,
                               GetPositionResponse *response) override;

    // GetPointCloudMap returns a stream of the current sampled pointcloud
    // derived from the painted map, using probability estimates in chunks with
    // a max size of maximumGRPCByteChunkSize
    ::grpc::Status GetPointCloudMap(
        ServerContext *context, const GetPointCloudMapRequest *request,
        ServerWriter<GetPointCloudMapResponse> *writer) override;

    // GetInternalState returns a stream of the current internal state of the
    // map which is a pbstream for cartographer in chunks of size
    // maximumGRPCByteChunkSize
    ::grpc::Status GetInternalState(
        ServerContext *context, const GetInternalStateRequest *request,
        ServerWriter<GetInternalStateResponse> *writer) override;

    // RunSLAM sets up and runs cartographer. It runs cartographer in
    // the ActionMode mode: Either creating
    // a new map, updating an apriori map, or localizing on an apriori map.
    void RunSLAM();

    // GetNextDataFile returns the next data file to be processed, determined
    // by whether cartographer is running in offline or online mode.
    std::string GetNextDataFile();

    // GetNextDataFileOffline returns the next data file in the directory.
    // Returns an empty string if done processing files or if stop has been
    // signaled.
    std::string GetNextDataFileOffline();

    // GetNextDataFileOfflineHelper is a helper function used to test
    // GetNextDataFileOffline
    std::string GetNextDataFileOfflineHelper();

    // GetNextDataFileOnline returns the most recently generated data that has
    // not been been processed, blocking if no new file is found. Returns an
    // empty string if stop has been signaled.
    std::string GetNextDataFileOnline();

    // GetActionMode returns the slam action mode from the provided
    // parameters.
    ActionMode GetActionMode();

    // SetActionMode sets the slam action mode based on provided
    // data and parameters.
    void SetActionMode();

    // OverwriteMapBuilderParameters overwrites cartographer specific
    // MapBuilder parameters.
    void OverwriteMapBuilderParameters();

    // Getter functions for map_builder parameters (called: options)
    int GetOptimizeEveryNNodesFromMapBuilder();
    int GetNumRangeDataFromMapBuilder();
    float GetMissingDataRayLengthFromMapBuilder();
    float GetMaxRangeFromMapBuilder();
    float GetMinRangeFromMapBuilder();
    int GetMaxSubmapsToKeepFromMapBuilder();
    int GetFreshSubmapsCountFromMapBuilder();
    double GetMinCoveredAreaFromMapBuilder();
    int GetMinAddedSubmapsCountFromMapBuilder();
    double GetOccupiedSpaceWeightFromMapBuilder();
    double GetTranslationWeightFromMapBuilder();
    double GetRotationWeightFromMapBuilder();

    std::string path_to_data;
    std::string path_to_map;
    std::string configuration_directory;
    std::string config_params;
    std::string port;
    std::string camera_name;
    std::chrono::milliseconds data_rate_ms;
    std::chrono::seconds map_rate_sec;
    std::string slam_mode;
    std::atomic<bool> optimize_on_start{false};
    std::atomic<bool> use_live_data{false};
    bool delete_processed_data = false;
    // The size of the buffer has to be the same as
    // dataBufferSize in RDK's builtin_test.go
    const int data_buffer_size = 4;
    int first_processed_file_index = -1;

    // -- Cartographer specific config params:
    // MAP_BUILDER.pose_graph
    int optimize_every_n_nodes = 3;
    // TRAJECTORY_BUILDER.trajectory_builder_2d.submaps
    int num_range_data = 100;
    // TRAJECTORY_BUILDER.trajectory_builder_2d
    float missing_data_ray_length = 25.0;
    float max_range = 25.0;
    float min_range = 0.2;
    // TRAJECTORY_BUILDER.pure_localization_trimmer
    int max_submaps_to_keep = 3;  // LOCALIZATION only
    // MAP_BUILDER.pose_graph.overlapping_submaps_trimmer_2d
    int fresh_submaps_count = 3;      // UPDATING only
    double min_covered_area = 1.0;    // UPDATING only
    int min_added_submaps_count = 1;  // UPDATING only
    // MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher
    double occupied_space_weight = 20.0;
    double translation_weight = 10.0;
    double rotation_weight = 1.0;

   private:
    // StartSaveMap starts the map saving process in a separate thread.
    void StartSaveMap();

    // StopSaveMap stops the map saving process that is running in a separate
    // thread.
    void StopSaveMap();

    // SaveMapWithTimestamp saves maps with a filename that includes the
    // timestamp of the time when the map is saved.
    void SaveMapWithTimestamp();

    // ConvertSavedMapToStream converted the saved pbstream to the passed in
    // string and deletes the file.
    void ConvertSavedMapToStream(const std::string filename_with_timestamp,
                                 std::string *buffer);

    // TryFileClose attempts to close an opened ifstream, returning an error
    // string if it fails.
    std::string TryFileClose(std::ifstream &file, std::string filename);

    // ProcessDataAndStartSavingMaps processes the data in the data directory
    // that is newer than the provided data_cutoff_time
    // and starts the process to save maps in parallel. In offline mode,
    // all data in the directory is processed. In online mode, the most
    // recently generated data is processed until a shutdown signal is
    // received.
    void ProcessDataAndStartSavingMaps(double data_cutoff_time);

    // SetUpMapBuilder loads the lua file with default cartographer config
    // parameters depending on the action mode. Setting the correct action
    // mode has to happen before calling this function.
    void SetUpMapBuilder();

    // SetUpSLAM sets the correct action mode, prepares the map builder and
    // loads the right hyperparameters based on the action mode. Needs to be
    // called before running slam.
    double SetUpSLAM();

    // GetLatestPaintedMapSlices paints and returns the current map of
    // Cartographer
    cartographer::io::PaintSubmapSlicesResult GetLatestPaintedMapSlices();

    // GetLatestSampledPointCloudMapString paints and returns the latest map as
    // a pcd string with probability estimates written to the color field. The
    // pcd is generated from PaintedMapSlices() and sampled to fit the 32 MB
    // limit on gRPC messages. The sampled behavior may change when moving to
    // streamed point clouds.
    void GetLatestSampledPointCloudMapString(std::string &pointcloud);

    // BackupLatestMap extracts and saves the latest map as a backup in
    // the respective member variables.
    void BackupLatestMap();

    // If using the LOCALIZING action mode, cache a copy of the map before
    // beginning to process data. If cartographer fails to do this,
    // terminate the program.
    void CacheMapInLocalizationMode();

    // GetPointCloudMap is a c compatible function
    int GetPointCloudMapC(viam_carto_get_point_cloud_map_response *response);

    ActionMode action_mode = ActionMode::MAPPING;

    const std::string configuration_mapping_basename = "mapping_new_map.lua";
    const std::string configuration_localization_basename =
        "locating_in_map.lua";
    const std::string configuration_update_basename = "updating_a_map.lua";

    std::vector<std::string> file_list_offline;
    size_t current_file_offline = 0;
    std::string max_file_offline;
    std::string current_file_online;

    // If mutexes map_builder_mutex and optimization_shared_mutex are held
    // concurrently, then optimization_shared_mutex must be taken
    // before map_builder_mutex. No other mutexes are expected to
    // be held concurrently.
    std::shared_mutex optimization_shared_mutex;
    std::mutex map_builder_mutex;
    mapping::MapBuilder map_builder;

    std::atomic<bool> finished_processing_offline{false};
    std::thread *thread_save_map_with_timestamp;

    std::mutex viam_response_mutex;
    cartographer::transform::Rigid3d latest_global_pose =
        cartographer::transform::Rigid3d();

    // The latest_pointcloud_map variable is used enable GetPointCloudMap to
    // send the most recent map out while cartographer works on creating an
    // optimized map. It is only updated right before the optimization is
    // started.
    std::string latest_pointcloud_map;
    // ---
};

}  // namespace viam

#endif  // SLAM_SERVICE_H_
