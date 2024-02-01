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

// BEGIN C API
#ifdef __cplusplus
extern "C" {
#endif

// Represents library level state
typedef struct viam_carto_lib {
    int minloglevel;
    int verbose;
} viam_carto_lib;

#define VIAM_CARTO_SLAM_MODE_UNKNOWN 0
#define VIAM_CARTO_SLAM_MODE_MAPPING 1
#define VIAM_CARTO_SLAM_MODE_LOCALIZING 2
#define VIAM_CARTO_SLAM_MODE_UPDATING 3

// Represents carto instance level state
typedef struct viam_carto {
    void *carto_obj;
    int slam_mode;
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
    bstring point_cloud_pcd;
} viam_carto_get_point_cloud_map_response;

typedef struct viam_carto_get_internal_state_response {
    bstring internal_state;
} viam_carto_get_internal_state_response;

typedef struct viam_carto_lidar_reading {
    bstring lidar;
    bstring lidar_reading;
    int64_t lidar_reading_time_unix_milli;
} viam_carto_lidar_reading;

typedef enum viam_carto_LIDAR_CONFIG {
    VIAM_CARTO_TWO_D = 0,
    VIAM_CARTO_THREE_D = 1
} viam_carto_LIDAR_CONFIG;

typedef struct viam_carto_imu_reading {
    bstring imu;
    double lin_acc_x;
    double lin_acc_y;
    double lin_acc_z;
    double ang_vel_x;
    double ang_vel_y;
    double ang_vel_z;
    int64_t imu_reading_time_unix_milli;
} viam_carto_imu_reading;

typedef struct viam_carto_odometer_reading {
    bstring odometer;
    double translation_x;
    double translation_y;
    double translation_z;
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
    int64_t odometer_reading_time_unix_milli;
} viam_carto_odometer_reading;

// return codes
#define VIAM_CARTO_SUCCESS 0
#define VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK 1
#define VIAM_CARTO_VC_INVALID 2
#define VIAM_CARTO_OUT_OF_MEMORY 3
#define VIAM_CARTO_DESTRUCTOR_ERROR 4
#define VIAM_CARTO_LIB_PLATFORM_INVALID 5
#define VIAM_CARTO_LIB_INVALID 6
#define VIAM_CARTO_LIB_NOT_INITIALIZED 7
#define VIAM_CARTO_UNKNOWN_ERROR 8
#define VIAM_CARTO_SLAM_MODE_INVALID 9
#define VIAM_CARTO_LIDAR_CONFIG_INVALID 10
#define VIAM_CARTO_COMPONENT_REFERENCE_INVALID 11
#define VIAM_CARTO_LUA_CONFIG_NOT_FOUND 12
#define VIAM_CARTO_INTERNAL_STATE_FILE_SYSTEM_ERROR 13
#define VIAM_CARTO_MAP_CREATION_ERROR 14
#define VIAM_CARTO_UNKNOWN_SENSOR_NAME 15
#define VIAM_CARTO_LIDAR_READING_EMPTY 16
#define VIAM_CARTO_LIDAR_READING_INVALID 17
#define VIAM_CARTO_GET_POSITION_RESPONSE_INVALID 18
#define VIAM_CARTO_GET_POSITION_NOT_INITIALIZED 19
#define VIAM_CARTO_POINTCLOUD_MAP_EMPTY 20
#define VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVALID 21
#define VIAM_CARTO_LIB_ALREADY_INITIALIZED 22
#define VIAM_CARTO_GET_INTERNAL_STATE_RESPONSE_INVALID 23
#define VIAM_CARTO_GET_INTERNAL_STATE_FILE_WRITE_IO_ERROR 24
#define VIAM_CARTO_GET_INTERNAL_STATE_FILE_READ_IO_ERROR 25
#define VIAM_CARTO_NOT_IN_INITIALIZED_STATE 26
#define VIAM_CARTO_NOT_IN_IO_INITIALIZED_STATE 27
#define VIAM_CARTO_NOT_IN_STARTED_STATE 28
#define VIAM_CARTO_NOT_IN_TERMINATABLE_STATE 29
#define VIAM_CARTO_IMU_PROVIDED_AND_IMU_ENABLED_MISMATCH 30
#define VIAM_CARTO_IMU_READING_EMPTY 31
#define VIAM_CARTO_IMU_READING_INVALID 32
#define VIAM_CARTO_ODOMETER_READING_INVALID 33

typedef struct viam_carto_algo_config {
    bool optimize_on_start;
    int optimize_every_n_nodes;
    int num_range_data;
    float missing_data_ray_length;
    float max_range;
    float min_range;
    bool use_imu_data;
    int max_submaps_to_keep;
    int fresh_submaps_count;
    double min_covered_area;
    int min_added_submaps_count;
    double occupied_space_weight;
    double translation_weight;
    double rotation_weight;
    bool has_initial_trajectory_pose = false;
    double initial_trajectory_pose_x;
    double initial_trajectory_pose_y;
    double initial_trajectory_pose_theta;

} viam_carto_algo_config;

typedef struct viam_carto_config {
    bstring camera;
    bstring movement_sensor;
    viam_carto_LIDAR_CONFIG lidar_config;
    bool enable_mapping;
    bstring existing_map;
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

// viam_carto_add_lidar_reading/3 takes a viam_carto pointer, a
// viam_carto_lidar_reading
//
// On error: Returns a non 0 error code
//
// An expected error is VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK(1)
//
// On success: Returns 0, adds lidar reading to cartographer's data model
extern int viam_carto_add_lidar_reading(viam_carto *vc,                     //
                                        const viam_carto_lidar_reading *sr  //
);

// viam_carto_add_lidar_reading_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_lidar_reading.
extern int viam_carto_add_lidar_reading_destroy(viam_carto_lidar_reading *sr  //
);

// viam_carto_add_imu_reading/3 takes a viam_carto pointer, a
// viam_carto_imu_reading
//
// On error: Returns a non 0 error code
//
// An expected error is VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK(1)
//
// On success: Returns 0, adds IMU reading to cartographer's data model
extern int viam_carto_add_imu_reading(viam_carto *vc,                   //
                                      const viam_carto_imu_reading *sr  //
);

// viam_carto_add_imu_reading_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_imu_reading.
extern int viam_carto_add_imu_reading_destroy(viam_carto_imu_reading *sr  //
);

// viam_carto_add_odometer_reading/3 takes a viam_carto pointer, a
// viam_carto_odometer_reading
//
// On error: Returns a non 0 error code
//
// An expected error is VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK(1)
//
// On success: Returns 0, adds odometer reading to cartographer's data model
extern int viam_carto_add_odometer_reading(
    viam_carto *vc,                        //
    const viam_carto_odometer_reading *sr  //
);

// viam_carto_add_odometer_reading_destroy/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0, frees the viam_carto_odometer_reading.
extern int viam_carto_add_odometer_reading_destroy(
    viam_carto_odometer_reading *sr  //
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

// viam_carto_run_final_optimization/2 takes a viam_carto pointer
//
// On error: Returns a non 0 error code
//
// On success: Returns 0 & blocks until all data has been processed
extern int viam_carto_run_final_optimization(viam_carto *vc);

#ifdef __cplusplus
}
#endif
// END C API

// BEGIN C++ API
#ifdef __cplusplus
namespace viam {
namespace carto_facade {
std::string to_std_string(bstring b_str);
enum class SlamMode { MAPPING, LOCALIZING, UPDATING };
std::ostream &operator<<(std::ostream &os, const SlamMode &slam_mode);
static const int checkForShutdownIntervalMicroseconds = 1e5;

// The resolutionMeters variable defines the area in meters that each pixel
// represents. This is used to draw the cairo map and in so doing defines the
// resolution of the outputted PCD
static const double resolutionMeters = 0.05;

typedef struct config {
    std::string camera;
    std::string movement_sensor;
    bstring component_reference;
    viam_carto_LIDAR_CONFIG lidar_config;
    bool enable_mapping;
    std::string existing_map;
} config;

// function to convert viam_carto_config into  viam::carto_facade::config
config from_viam_carto_config(viam_carto_config vcc);

// Error log for when no submaps exist
static const std::string errorNoSubmaps = "No submaps to paint";

const std::string configuration_mapping_basename = "mapping_new_map.lua";
const std::string configuration_localization_basename = "locating_in_map.lua";
const std::string configuration_update_basename = "updating_a_map.lua";

carto_facade::SlamMode determine_slam_mode(std::string path_to_map,
                                           bool enable_mapping);

int slam_mode_to_vc_slam_mode(viam::carto_facade::SlamMode sm);

enum class CartoFacadeState { INITIALIZED, IO_INITIALIZED, STARTED };
class CartoFacade {
   public:
    CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                const viam_carto_algo_config ac);
    ~CartoFacade();

    // IOInit:
    // 1. sets the correct slam mode
    // 2. creates & configures the map builder with the right hyperparameters
    // based on the slam mode
    // 3. starts the trajectory builder
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

    void AddLidarReading(const viam_carto_lidar_reading *sr);

    void AddIMUReading(const viam_carto_imu_reading *sr);

    void AddOdometerReading(const viam_carto_odometer_reading *sr);

    void Start();

    void Stop();

    // non api methods
    void CacheLatestMap();
    void CacheMapInLocalizationMode();
    void GetLatestSampledPointCloudMapString(std::string &pointcloud);
    void RunFinalOptimization();
    cartographer::io::PaintSubmapSlicesResult GetLatestPaintedMapSlices();
    viam_carto_lib *lib;
    viam::carto_facade::config config;
    viam_carto_algo_config algo_config;
    std::string path_to_internal_state_file;
    std::atomic<CartoFacadeState> state{CartoFacadeState::INITIALIZED};
    std::string configuration_directory;
    SlamMode slam_mode = SlamMode::MAPPING;

    // If mutexes map_builder_mutex and optimization_shared_mutex are held
    // concurrently, then optimization_shared_mutex must be taken
    // before map_builder_mutex. No other mutexes are expected to
    // be held concurrently.
    std::mutex map_builder_mutex;
    MapBuilder map_builder;

   private:
    // moved from namespace
    std::shared_mutex optimization_shared_mutex;

    std::unique_ptr<std::thread> thread_save_internal_state;

    std::mutex viam_response_mutex;
    cartographer::transform::Rigid3d latest_global_pose =
        cartographer::transform::Rigid3d();
    // The latest_pointcloud_map variable is used to enable GetPointCloudMap to
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
