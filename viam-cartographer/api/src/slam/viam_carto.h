#ifndef VIAM_CARTO_H
#define VIAM_CARTO_H
// includes stdlib.h
#include "bstrlib.h"
/* #include "viam_carto_string.h" */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
// TODO remove unused
#define UNUSED(x) (void)(x)

typedef struct viam_carto {
  bstring *sensors;
  int sensors_len;
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

  const char *component_reference;
  // TODO: Need to also return quat information as the spaital math exists only
  // on the go side
} viam_carto_get_position_response;

typedef struct viam_carto_get_point_cloud_map_response {
  bstring point_cloud_pcd;
} viam_carto_get_point_cloud_map_response;

typedef struct viam_carto_get_internal_state_response {
  bstring internal_state;
} viam_carto_get_internal_state_response;

typedef struct viam_carto_sensor_reading {
  bstring sensor;
  bstring sensor_reading;
  uint64_t sensor_reading_time_unix_micro;
} viam_carto_sensor_reading;

typedef enum viam_carto_MODE {
  VIAM_CARTO_LOCALIZING = 0,
  VIAM_CARTO_MAPPING = 1,
  VIAM_CARTO_UPDATING = 2
} viam_carto_MODE;

typedef enum viam_carto_LIDAR_CONFIG {
  TWO_D = 0,
  THREE_D = 1
} viam_carto_LIDAR_CONFIG;

// return codes
#define VIAM_CARTO_SUCCESS 0
#define VIAM_CARTO_UNABLE_TO_AQUIRE_LOCK 1
#define VIAM_CARTO_VC_INVALID 2
#define VIAM_CARTO_OUT_OF_MEMORY 3
#define VIAM_CARTO_DESTRUCTOR_ERROR 4

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
  bstring *sensors;
  int sensors_len;
  int map_rate_sec;
  bstring data_dir;
  bstring component_reference;
  viam_carto_MODE mode;
  viam_carto_LIDAR_CONFIG lidar_config;
} viam_carto_config;

// viam_carto_init/4 takes a null viam_carto pointer and a viam_carto_config, a
// viam_carto_algo_config, and empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0 & mutates viam_carto to contain handle to
// initialized carto object
int viam_carto_init(viam_carto **vc,                 // OUT
                    const viam_carto_config c,       //
                    const viam_carto_algo_config ac, //
                    char **errmsg                    // OUT
);

// viam_carto_start/2 takes a viam_carto pointer and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, starts cartographer
int viam_carto_start(viam_carto **vc, // OUT
                     char **errmsg    // OUT
);

// viam_carto_stop/2 takes a viam_carto pointer and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, stops work begun by viam_carto_start()
int viam_carto_stop(viam_carto **vc, // OUT
                    char **errmsg    // OUT
);

// viam_carto_stop/2 takes a viam_carto pointer and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, frees all resources aquired by
// viam_carto_init/4
int viam_carto_terminate(viam_carto **vc, //
                         char **errmsg    // OUT
);

// viam_carto_add_sensor_reading/3 takes a viam_carto pointer, a
// viam_carto_sensor_reading and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// An expected error is VIAM_CARTO_UNABLE_TO_AQUIRE_LOCK(1)
//
// On success: Returns 0, adds lidar reading to cartographer's data model
int viam_carto_add_sensor_reading(const viam_carto **vc,               //
                                  const viam_carto_sensor_reading *sr, //
                                  char **errmsg                        // OUT
);

// viam_carto_add_sensor_reading_destroy/2 takes a viam_carto pointer and an
// empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, frees the viam_carto_sensor_reading.
int viam_carto_add_sensor_reading_destroy(viam_carto_sensor_reading *sr, //
                                          char **errmsg                  // OUT
);

// viam_carto_get_position/3 takes a viam_carto pointer, a
// viam_carto_get_position_response pointer and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, mutates viam_carto_get_position_response
// to contain the response
int viam_carto_get_position(const viam_carto **vc,               //
                            viam_carto_get_position_response *r, // OUT
                            char **errmsg                        // OUT
);

// viam_carto_get_position_response_destroy/2 takes a viam_carto pointer and an
// empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, frees the viam_carto_get_position_response.
int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r, //
    char **errmsg                        // OUT
);

// viam_carto_get_point_cloud_map/3 takes a viam_carto pointer, a
// viam_carto_get_point_cloud_map_response pointer and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, mutates viam_carto_get_point_cloud_map_response
// to contain the response
int viam_carto_get_point_cloud_map(
    const viam_carto **vc,                      //
    viam_carto_get_point_cloud_map_response *r, // OUT
    char **errmsg                               // OUT
);

// viam_carto_get_point_cloud_map_response_destroy/2 takes a viam_carto pointer
// and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, frees the viam_carto_get_point_cloud_map_response.
int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r, //
    char **errmsg                               // OUT
);

// viam_carto_get_internal_state/3 takes a viam_carto pointer, a
// viam_carto_get_internal_state_response pointer and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, mutates viam_carto_get_internal_state_response
// to contain the response
int viam_carto_get_internal_state(
    const viam_carto **vc,                     //
    viam_carto_get_internal_state_response *r, // OUT
    char **errmsg                              // OUT
);

// viam_carto_get_internal_state_response_destroy/2 takes a viam_carto pointer
// and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message.
//
// On success: Returns 0, frees the viam_carto_get_internal_state_response.
int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r, //
    char **errmsg                              // OUT
);
#endif /*VIAM_CARTO_H*/
