#ifndef VIAM_CARTO_H
#define VIAM_CARTO_H
/*
 * Modeled after sqlite api:
 * https://www.sqlite.org/cintro.html
 *
 * NOTE:
 * I see that SQLITE uses pointers to pointers for paramenters.
 * I'm not sure why. I've copied the pattern here, lets have a convo
 * regarding whether that makes sense to do or not.
 */

/*
 * Arguments sent to carto_grpc_server on initialization now
 * "-sensors="+cartoSvc.primarySensorName
 * "-config_param="+slamUtils.DictToString(cartoSvc.configParams))
 * "-data_rate_ms="+strconv.Itoa(cartoSvc.dataRateMs))
 * "-map_rate_sec="+strconv.Itoa(cartoSvc.mapRateSec))
 * "-data_dir="+cartoSvc.dataDirectory)
 * "-delete_processed_data="+strconv.FormatBool(cartoSvc.deleteProcessedData))
 * "-use_live_data="+strconv.FormatBool(cartoSvc.useLiveData))
 * "-port="+cartoSvc.port)
 * "--aix-auto-update")
 */

/* Arguments Nick thinks should be sent on initialization after modularization
 * v1
 * "-sensors="+cartoSvc.primarySensorName
 * NOTE: This should be split out into its individual parameters
 * "-config_param="+slamUtils.DictToString(cartoSvc.configParams))
 * "-map_rate_sec="+strconv.Itoa(cartoSvc.mapRateSec))
 * "-data_dir="+cartoSvc.dataDirectory)
 */

typedef struct viam_carto {
  const char *sensors;
  void *carto_obj;
} viam_carto;

// GetPositionResponse
// https://github.com/viamrobotics/api/blob/main/proto/viam/service/slam/v1/slam.proto#L51
typedef struct viam_carto_get_position_response {
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
  const char *component_reference;
  // TODO: Need to also return quat information as the spaital math exists only
  // on the go side
} viam_carto_get_position_response;

typedef struct viam_carto_get_point_cloud_map_response {
  const char *point_cloud_pcd;
  const int point_cloud_pcd_len;
} viam_carto_get_point_cloud_map_response;

typedef struct viam_carto_get_internal_state_response {
  const char *internal_state;
  const int internal_state_len;
} viam_carto_get_internal_state_response;

typedef struct viam_carto_sensor_reading {
  int sensor_reading_len;
  char *sensor;
  char *sensor_reading;
  // TODO Research how to represent time
  /* char *time; */
} viam_carto_sensor_reading;

enum viam_carto_MODE {
  VIAM_CARTO_LOCALIZING = 0,
  VIAM_CARTO_MAPPING = 1,
  VIAM_CARTO_UPDATING = 2
};
enum viam_carto_LIDAR_CONFIG { TWO_D = 0, THREE_D = 1 };
enum viam_carto_ERROR { VIAM_CARTO_SUCCESS = 0 };

typedef struct viam_carto_algo_config {
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
  const char **sensors;
  int sensors_len;
  int map_rate_sec;
  const char *data_dir;
  const char *component_reference;
  viam_carto_MODE mode;
  viam_carto_LIDAR_CONFIG lidar_config;
} viam_carto_config;

viam_carto_ERROR viam_carto_init(viam_carto *vc,                  // OUT
                                 const viam_carto_config c,       //
                                 const viam_carto_algo_config ac, //
                                 char **errmsg                    // OUT
);

viam_carto_ERROR viam_carto_start(viam_carto *vc, // OUT
                                  char **errmsg   // OUT
);
viam_carto_ERROR viam_carto_stop(viam_carto *vc, // OUT
                                 char **errmsg   // OUT
);
viam_carto_ERROR viam_carto_terminate(viam_carto *vc, //
                                      char **errmsg   // OUT
);

// add_sensor_reading
viam_carto_ERROR
viam_carto_add_sensor_reading(const viam_carto *vc,                //
                              const viam_carto_sensor_reading *sr, //
                              char **errmsg                        // OUT
);
// sensor_reading destructor
viam_carto_ERROR
viam_carto_add_sensor_reading_destroy(viam_carto_sensor_reading *sr, //
                                      char **errmsg                  // OUT
);

// get_position
viam_carto_ERROR
viam_carto_get_position(const viam_carto *vc,                //
                        viam_carto_get_position_response *r, // OUT
                        char **errmsg                        // OUT
);

// get_position_response destructor
viam_carto_ERROR
viam_carto_get_position_response_destroy(viam_carto_get_position_response *r, //
                                         char **errmsg // OUT
);

// get_point_cloud_map
viam_carto_ERROR viam_carto_get_point_cloud_map(
    const viam_carto *vc,                       //
    viam_carto_get_point_cloud_map_response *r, // OUT
    char **errmsg                               // OUT
);

// get_position_response destructor
viam_carto_ERROR viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r, //
    char **errmsg                               // OUT
);

// get_internal_state
viam_carto_ERROR
viam_carto_get_internal_state(const viam_carto *vc,                      //
                              viam_carto_get_internal_state_response *r, // OUT
                              char **errmsg                              // OUT
);

// get_internal_state_response destructor
viam_carto_ERROR viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r, //
    char **errmsg                              // OUT
);
#endif /*VIAM_CARTO_H*/
