// This is an experimental integration of cartographer into RDK.
#ifndef CARTO_FACADE_H
#define CARTO_FACADE_H

#include <atomic>
#include <chrono>
#include <shared_mutex>
#include <string>

#include "bstrlib.h"
#include "bstrwrap.h"
#include "map_builder.h"

// #include "../io/draw_trajectories.h"
// #include "../io/file_handler.h"
// #include "../utils/slam_service_helpers.h"
// #include "Eigen/Core"
// #include "cairo/cairo.h"
// #include "cartographer/io/submap_painter.h"

// BEGIN C API
#ifdef __cplusplus
extern "C" {
#endif

// Represents library level state
typedef struct viam_carto_lib {
    int minloglevel;
    int verbose;
} viam_carto_lib;

// Represents carto instance level state
typedef struct viam_carto {
    void *carto_obj;
} viam_carto;

// GetPositionResponse
// https://github.com/viamrobotics/api/blob/main/proto/viam/service/slam/v1/slam.proto#L51
typedef struct viam_carto_get_position_response {
    // millimeters from the origin
    double x;
    // millimeters from the origin
    double y;
    // millimeters from the origin
    double z;
    // z component of a vector defining axis of rotation
    double o_x;
    // x component of a vector defining axis of rotation
    double o_y;
    // y component of a vector defining axis of rotation
    double o_z;
    // degrees
    double theta;

    // Quaternian information
    double real;
    double imag;
    double jmag;
    double kmag;

    bstring component_reference;
} viam_carto_get_position_response;

typedef struct viam_carto_get_point_cloud_map_response {
    // TODO: change to void* and a size
    bstring point_cloud_pcd;
} viam_carto_get_point_cloud_map_response;

typedef struct viam_carto_get_internal_state_response {
    // TODO: change to void* and a size
    bstring internal_state;
} viam_carto_get_internal_state_response;

typedef struct viam_carto_sensor_reading {
    bstring sensor;
    // TODO: change to void* and a size
    bstring sensor_reading;
    uint64_t sensor_reading_time_unix_micro;
} viam_carto_sensor_reading;

typedef enum viam_carto_MODE {
    VIAM_CARTO_LOCALIZING = 0,
    VIAM_CARTO_MAPPING = 1,
    VIAM_CARTO_UPDATING = 2
} viam_carto_MODE;

typedef enum viam_carto_LIDAR_CONFIG {
    VIAM_CARTO_TWO_D = 0,
    VIAM_CARTO_THREE_D = 1
} viam_carto_LIDAR_CONFIG;

// return codes
#define VIAM_CARTO_SUCCESS 0
#define VIAM_CARTO_UNABLE_TO_AQUIRE_LOCK 1
#define VIAM_CARTO_VC_INVALID 2
#define VIAM_CARTO_OUT_OF_MEMORY 3
#define VIAM_CARTO_DESTRUCTOR_ERROR 4
#define VIAM_CARTO_LIB_PLATFORM_INVALID 5
#define VIAM_CARTO_LIB_INVALID 6
#define VIAM_CARTO_LIB_NOT_INITIALIZED 7
#define VIAM_CARTO_SENSORS_LIST_EMPTY 8
#define VIAM_CARTO_UNKNOWN_ERROR 9
#define VIAM_CARTO_DATA_DIR_NOT_PROVIDED 10
#define VIAM_CARTO_SLAM_MODE_INVALID 11
#define VIAM_CARTO_LIDAR_CONFIG_INVALID 12
#define VIAM_CARTO_MAP_RATE_SEC_INVALID 13
#define VIAM_CARTO_COMPONENT_REFERENCE_INVALID 14
#define VIAM_CARTO_LUA_CONFIG_NOT_FOUND 15

typedef struct viam_carto_algo_config {
    bool optimize_on_start;
    int optimize_every_n_nodes;
    int num_range_data;
    float missing_data_ray_length;
    float max_range;
    float min_range;
    int max_submaps_to_keep;
    int fresh_submaps_count;
    double min_covered_area;
    int min_added_submaps_count;
    double occupied_space_weight;
    double translation_weight;
    double rotation_weight;
} viam_carto_algo_config;

typedef struct viam_carto_config {
    bstring *sensors;
    int sensors_len;
    int map_rate_sec;
    bstring data_dir;
    bstring component_reference;
    viam_carto_MODE mode;
    viam_carto_LIDAR_CONFIG lidar_config;
} viam_carto_config;

// viam_carto_lib_init/4 takes an empty viam_carto_lib pointer to pointer
// On error: Returns a non 0 error code
//
// On success: Returns 0 & mutates viam_carto_lib to contain a handle to
// the initialized library state.
extern int viam_carto_lib_init(viam_carto_lib **vcl,  // OUT
                               int minloglevel, int verbose);

// viam_carto_lib_terminate/4 takes a valid viam_carto_lib pointer to pointer
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees all resources aquired by
// viam_carto_lib_init/4
extern int viam_carto_lib_terminate(viam_carto_lib **vcl  // OUT
);

// viam_carto_init/4 takes an empty viam_carto pointer to pointer,
// a viam_carto_lib pointer and a viam_carto_config, and a
// viam_carto_algo_config
// On error: Returns a non 0 error code
//
// On success: Returns 0 & mutates viam_carto to contain handle to
// initialized carto object
extern int viam_carto_init(viam_carto **vc,            // OUT
                           viam_carto_lib *pVCL,       //
                           const viam_carto_config c,  //
                           const viam_carto_algo_config ac);

// viam_carto_start/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, starts cartographer
extern int viam_carto_start(viam_carto *vc  // OUT
);

// viam_carto_stop/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, stops work begun by viam_carto_start()
extern int viam_carto_stop(viam_carto *vc  // OUT
);

// viam_carto_terminate/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees all resources aquired by
// viam_carto_init/4
extern int viam_carto_terminate(viam_carto **vc  //
);

// viam_carto_add_sensor_reading/3 takes a viam_carto pointer, a
// viam_carto_sensor_reading
//
// On error: Returns a non 0 error code
//
// An expected error is VIAM_CARTO_UNABLE_TO_AQUIRE_LOCK(1)
//
// On success: Returns 0, adds lidar reading to cartographer's data model
extern int viam_carto_add_sensor_reading(viam_carto *vc,                      //
                                         const viam_carto_sensor_reading *sr  //
);

// viam_carto_add_sensor_reading_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_sensor_reading.
extern int viam_carto_add_sensor_reading_destroy(
    viam_carto_sensor_reading *sr  //
);

// viam_carto_get_position/3 takes a viam_carto pointer, a
// viam_carto_get_position_response pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, mutates viam_carto_get_position_response
// to contain the response
extern int viam_carto_get_position(viam_carto *vc,                      //
                                   viam_carto_get_position_response *r  // OUT
);

// viam_carto_get_position_response_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_get_position_response.
extern int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r  //
);

// viam_carto_get_point_cloud_map/3 takes a viam_carto pointer, a
// viam_carto_get_point_cloud_map_response pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, mutates viam_carto_get_point_cloud_map_response
// to contain the response
extern int viam_carto_get_point_cloud_map(
    viam_carto *vc,                             //
    viam_carto_get_point_cloud_map_response *r  // OUT
);

// viam_carto_get_point_cloud_map_response_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_get_point_cloud_map_response.
extern int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r  //
);

// viam_carto_get_internal_state/3 takes a viam_carto pointer, a
// viam_carto_get_internal_state_response pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, mutates viam_carto_get_internal_state_response
// to contain the response
extern int viam_carto_get_internal_state(
    viam_carto *vc,                            //
    viam_carto_get_internal_state_response *r  // OUT
);

// viam_carto_get_internal_state_response_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_get_internal_state_response.
extern int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r);

#ifdef __cplusplus
}
#endif
// END C API

// BEGIN C++ API
#ifdef __cplusplus
namespace viam {
namespace carto_facade {
enum class ActionMode { MAPPING, LOCALIZING, UPDATING };
static const int checkForShutdownIntervalMicroseconds = 1e5;

// The resolutionMeters variable defines the area in meters that each pixel
// represents. This is used to draw the cairo map and in so doing defines the
// resolution of the outputted PCD
static const double resolutionMeters = 0.05;

typedef struct config {
    std::vector<std::string> sensors;
    std::chrono::seconds map_rate_sec;
    std::string data_dir;
    std::string component_reference;
    viam_carto_MODE mode;
    viam_carto_LIDAR_CONFIG lidar_config;
} config;

// function to convert viam_carto_config into  viam::carto_facade::config
config from_viam_carto_config(viam_carto_config vcc);

// Error log for when no submaps exist
// std::string errorNoSubmaps = "No submaps to paint";

const std::string configuration_mapping_basename = "mapping_new_map.lua";
const std::string configuration_localization_basename = "locating_in_map.lua";
const std::string configuration_update_basename = "updating_a_map.lua";

carto_facade::ActionMode determine_action_mode(
    std::string path_to_map, std::chrono::seconds map_rate_sec);
/* std::string get_latest_map_filename(std::string path_to_map); */
/* std::string pcd_header(int mapSize, bool hasColor); */
/* void write_float_to_buffer_in_bytes(std::string &buffer, float f); */
/* void write_int_to_buffer_in_bytes(std::string &buffer, int d); */

class CartoFacade {
   public:
    CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                const viam_carto_algo_config ac);

    int IOInit();

    // GetPosition returns the relative pose of the robot w.r.t the "origin"
    // of the map, which is the starting point from where the map was initially
    // created along with a component reference.
    int GetPosition(viam_carto_get_position_response *r);

    // GetPointCloudMap returns a stream of the current sampled pointcloud
    // derived from the painted map, using probability estimates in chunks with
    // a max size of maximumGRPCByteChunkSize
    int GetPointCloudMap(viam_carto_get_point_cloud_map_response *r);

    // GetInternalState returns a stream of the current internal state of the
    // map which is a pbstream for cartographer in chunks of size
    // maximumGRPCByteChunkSize
    int GetInternalState(viam_carto_get_internal_state_response *r);

    int AddSensorReading(viam_carto_sensor_reading *sr);

    int Start();

    int Stop();

    // RunSLAM sets up and runs cartographer. It runs cartographer in
    // the ActionMode mode: Either creating
    // a new map, updating an apriori map, or localizing on an apriori map.
    // void RunSLAM();

    // GetNextDataFile returns the next data file to be processed, determined
    // by whether cartographer is running in offline or online mode.
    // std::string GetNextDataFile();

    // GetNextDataFileOffline returns the next data file in the directory.
    // Returns an empty string if done processing files or if stop has been
    // signaled.
    // std::string GetNextDataFileOffline();

    // GetNextDataFileOnline returns the most recently generated data that has
    // not been been processed, blocking if no new file is found. Returns an
    // empty string if stop has been signaled.
    // std::string GetNextDataFileOnline();

    // GetActionMode returns the slam action mode from the provided
    // parameters.
    // ActionMode GetActionMode();

    // SetActionMode sets the slam action mode based on provided
    // data and parameters.
    // void SetActionMode();

    // OverwriteMapBuilderParameters overwrites cartographer specific
    // MapBuilder parameters.
    // void OverwriteMapBuilderParameters();

    // Getter functions for map_builder parameters (called: options)
    // int GetOptimizeEveryNNodesFromMapBuilder();
    // int GetNumRangeDataFromMapBuilder();
    // float GetMissingDataRayLengthFromMapBuilder();
    // float GetMaxRangeFromMapBuilder();
    // float GetMinRangeFromMapBuilder();
    // int GetMaxSubmapsToKeepFromMapBuilder();
    // int GetFreshSubmapsCountFromMapBuilder();
    // double GetMinCoveredAreaFromMapBuilder();
    // int GetMinAddedSubmapsCountFromMapBuilder();
    // double GetOccupiedSpaceWeightFromMapBuilder();
    // double GetTranslationWeightFromMapBuilder();
    // double GetRotationWeightFromMapBuilder();

    // The size of the buffer has to be the same as
    // dataBufferSize in RDK's builtin_test.go
    // const int data_buffer_size = 4;
    // int first_processed_file_index = -1;

    // -- Cartographer specific config params:
    // MAP_BUILDER.pose_graph
    // int optimize_every_n_nodes = 3;
    // TRAJECTORY_BUILDER.trajectory_builder_2d.submaps
    // int num_range_data = 100;
    // TRAJECTORY_BUILDER.trajectory_builder_2d
    // float missing_data_ray_length = 25.0;
    // float max_range = 25.0;
    // float min_range = 0.2;
    // TRAJECTORY_BUILDER.pure_localization_trimmer
    // int max_submaps_to_keep = 3;  // LOCALIZATION only
    // MAP_BUILDER.pose_graph.overlapping_submaps_trimmer_2d
    // int fresh_submaps_count = 3;      // UPDATING only
    // double min_covered_area = 1.0;    // UPDATING only
    // int min_added_submaps_count = 1;  // UPDATING only
    // MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher
    // double occupied_space_weight = 20.0;
    // double translation_weight = 10.0;
    // double rotation_weight = 1.0;

    viam_carto_lib *lib;
    viam::carto_facade::config config;
    viam_carto_algo_config algo_config;
    std::string path_to_data;
    std::string path_to_internal_state;
    std::atomic<bool> b_continue_session;
    std::string configuration_directory;

    std::mutex map_builder_mutex;
    MapBuilder map_builder;
    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder;
    int trajectory_id;
    ActionMode action_mode = ActionMode::MAPPING;

   private:
    // moved from namespace
    // StartSaveMap starts the map saving process in a separate thread.
    // void StartSaveMap();

    // StopSaveMap stops the map saving process that is running in a separate
    // thread.
    // void StopSaveMap();

    // SaveMapWithTimestamp saves maps with a filename that includes the
    // timestamp of the time when the map is saved.
    // void SaveMapWithTimestamp();

    // ConvertSavedMapToStream converted the saved pbstream to the passed in
    // string and deletes the file.
    // void ConvertSavedMapToStream(const std::string filename_with_timestamp,
    //                              std::string *buffer);

    // TryFileClose attempts to close an opened ifstream, returning an error
    // string if it fails.
    // std::string TryFileClose(std::ifstream &file, std::string filename);

    // ProcessDataAndStartSavingMaps processes the data in the data directory
    // that is newer than the provided data_cutoff_time
    // and starts the process to save maps in parallel. In offline mode,
    // all data in the directory is processed. In online mode, the most
    // recently generated data is processed until a shutdown signal is
    // received.
    // void ProcessDataAndStartSavingMaps(double data_cutoff_time);

    // SetUpMapBuilder loads the lua file with default cartographer config
    // parameters depending on the action mode. Setting the correct action
    // mode has to happen before calling this function.
    // void SetUpMapBuilder();

    // SetUpSLAM sets the correct action mode, prepares the map builder and
    // loads the right hyperparameters based on the action mode. Needs to be
    // called before running slam.
    // double SetUpSLAM();

    // GetLatestPaintedMapSlices paints and returns the current map of
    // Cartographer
    // cartographer::io::PaintSubmapSlicesResult GetLatestPaintedMapSlices();

    // GetLatestSampledPointCloudMapString paints and returns the latest map as
    // a pcd string with probability estimates written to the color field. The
    // pcd is generated from PaintedMapSlices() and sampled to fit the 32 MB
    // limit on gRPC messages. The sampled behavior may change when moving to
    // streamed point clouds.
    // void GetLatestSampledPointCloudMapString(std::string &pointcloud);

    // BackupLatestMap extracts and saves the latest map as a backup in
    // the respective member variables.
    // void BackupLatestMap();

    // If using the LOCALIZING action mode, cache a copy of the map before
    // beginning to process data. If cartographer fails to do this,
    // terminate the program.
    // void CacheMapInLocalizationMode();

    // std::vector<std::string> file_list_offline;
    // size_t current_file_offline = 0;
    // std::string current_file_online;

    // If mutexes map_builder_mutex and optimization_shared_mutex are held
    // concurrently, then optimization_shared_mutex must be taken
    // before map_builder_mutex. No other mutexes are expected to
    // be held concurrently.
    // std::shared_mutex optimization_shared_mutex;

    // std::atomic<bool> finished_processing_offline{false};
    // std::thread *thread_save_map_with_timestamp;

    // std::mutex viam_response_mutex;
    // cartographer::transform::Rigid3d latest_global_pose =
    //     cartographer::transform::Rigid3d();

    // The latest_pointcloud_map variable is used enable GetPointCloudMap to
    // send the most recent map out while cartographer works on creating an
    // optimized map. It is only updated right before the optimization is
    // started.
    // std::string latest_pointcloud_map;
    // ---
};
}  // namespace carto_facade
}  // namespace viam

#else
typedef struct CartoFacade CartoFacade;
#endif
// END C++ API

#endif  // CARTO_FACADE_H
