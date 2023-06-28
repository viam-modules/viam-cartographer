// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTO_FACADE_H
#define VIAM_CARTO_FACADE_H

#ifdef __cplusplus
#include <atomic>
#include <chrono>
#include <shared_mutex>
#include <string>

#include "cartographer/io/submap_painter.h"
#include "map_builder.h"
#else
#include <stdbool.h>
#include <stdint.h>
#endif

#include "bstrlib.h"
#include "bstrwrap.h"

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
    int64_t sensor_reading_time_unix_micro;
} viam_carto_sensor_reading;

typedef enum viam_carto_LIDAR_CONFIG {
    VIAM_CARTO_TWO_D = 0,
    VIAM_CARTO_THREE_D = 1
} viam_carto_LIDAR_CONFIG;

// return codes
#define VIAM_CARTO_SUCCESS 0
#define VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK 1
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
#define VIAM_CARTO_DATA_DIR_INVALID_DEPRECATED_STRUCTURE 16
#define VIAM_CARTO_DATA_DIR_FILE_SYSTEM_ERROR 17
#define VIAM_CARTO_MAP_CREATION_ERROR 18
#define VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST 19
#define VIAM_CARTO_SENSOR_READING_EMPTY 20
#define VIAM_CARTO_SENSOR_READING_INVALID 21
#define VIAM_CARTO_GET_POSITION_RESPONSE_INVALID 22
#define VIAM_CARTO_POINTCLOUD_MAP_EMPTY 23

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
// An expected error is VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK(1)
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
std::string to_std_string(bstring b_str);
enum class ActionMode { MAPPING, LOCALIZING, UPDATING };
std::ostream &operator<<(std::ostream &os, const ActionMode &action_mode);
static const int checkForShutdownIntervalMicroseconds = 1e5;

// The resolutionMeters variable defines the area in meters that each pixel
// represents. This is used to draw the cairo map and in so doing defines the
// resolution of the outputted PCD
static const double resolutionMeters = 0.05;

typedef struct config {
    std::vector<std::string> sensors;
    std::chrono::seconds map_rate_sec;
    std::string data_dir;
    bstring component_reference;
    viam_carto_LIDAR_CONFIG lidar_config;
} config;

// function to convert viam_carto_config into  viam::carto_facade::config
config from_viam_carto_config(viam_carto_config vcc);

// Error log for when no submaps exist
static const std::string errorNoSubmaps = "No submaps to paint";

const std::string configuration_mapping_basename = "mapping_new_map.lua";
const std::string configuration_localization_basename = "locating_in_map.lua";
const std::string configuration_update_basename = "updating_a_map.lua";

carto_facade::ActionMode determine_action_mode(
    std::string path_to_map, std::chrono::seconds map_rate_sec);

class CartoFacade {
   public:
    CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                const viam_carto_algo_config ac);
    ~CartoFacade();

    // IOInit:
    // 1. detects if the data_dir has a deprecated format & throws if it does
    // 2. creates the data_dir with the correct format if it doesn't exist
    // 3. sets the correct action mode
    // 4. creates & configures the map builder with the right hyperparameters
    // based on the action mode
    // 5. starts the trajectory builder
    //
    // Needs to be first method called on newly instantiated CartoFacade object.
    void IOInit();

    // GetPosition returns the relative pose of the robot w.r.t the "origin"
    // of the map, which is the starting point from where the map was initially
    // created along with a component reference.
    void GetPosition(viam_carto_get_position_response *r);

    // GetPointCloudMap returns a stream of the current sampled pointcloud
    // derived from the painted map, using probability estimates in chunks with
    // a max size of maximumGRPCByteChunkSize
    void GetPointCloudMap(viam_carto_get_point_cloud_map_response *r);

    // GetInternalState returns a stream of the current internal state of the
    // map which is a pbstream for cartographer in chunks of size
    // maximumGRPCByteChunkSize
    void GetInternalState(viam_carto_get_internal_state_response *r);

    void AddSensorReading(const viam_carto_sensor_reading *sr);

    void Start();

    void Stop();

    // non api methods
    void CacheLatestMap();
    void CacheMapInLocalizationMode();
    void GetLatestSampledPointCloudMapString(std::string &pointcloud);
    cartographer::io::PaintSubmapSlicesResult GetLatestPaintedMapSlices();
    viam_carto_lib *lib;
    viam::carto_facade::config config;
    viam_carto_algo_config algo_config;
    std::string path_to_internal_state;
    std::atomic<bool> started{false};
    std::string configuration_directory;
    ActionMode action_mode = ActionMode::MAPPING;

    // If mutexes map_builder_mutex and optimization_shared_mutex are held
    // concurrently, then optimization_shared_mutex must be taken
    // before map_builder_mutex. No other mutexes are expected to
    // be held concurrently.
    std::mutex map_builder_mutex;
    MapBuilder map_builder;

   private:
    // moved from namespace
    // StartSaveInternalState starts the map saving process in a separate
    // thread.
    void StartSaveInternalState();

    // StopSaveInternalState stops the map saving process that is running in a
    // separate thread.
    void StopSaveInternalState();

    // SaveInternalStateOnInterval saves internal state with a filename that
    // includes the timestamp of the time when the map is saved.
    void SaveInternalStateOnInterval();

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

    // GetLatestPaintedMapSlices paints and returns the current map of
    // Cartographer
    // cartographer::io::PaintSubmapSlicesResult GetLatestPaintedMapSlices();

    // GetLatestSampledPointCloudMapString paints and returns the latest map as
    // a pcd string with probability estimates written to the color field. The
    // pcd is generated from PaintedMapSlices() and sampled to fit the 32 MB
    // limit on gRPC messages. The sampled behavior may change when moving to
    // streamed point clouds.
    // void GetLatestSampledPointCloudMapString(std::string &pointcloud);

    // CacheLatestMap extracts and saves the latest map as a backup in
    // the respective member variables.
    // void CacheLatestMap();

    // If using the LOCALIZING action mode, cache a copy of the map before
    // beginning to process data. If cartographer fails to do this,
    // terminate the program.
    // void CacheMapInLocalizationMode();

    // std::vector<std::string> file_list_offline;
    // size_t current_file_offline = 0;
    // std::string current_file_online;

    std::shared_mutex optimization_shared_mutex;

    // std::atomic<bool> finished_processing_offline{false};
    std::unique_ptr<std::thread> thread_save_internal_state;

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
}  // namespace carto_facade
}  // namespace viam

#else
typedef struct CartoFacade CartoFacade;
#endif
// END C++ API

#endif  // VIAM_CARTO_FACADE_H
