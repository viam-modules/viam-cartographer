#include <stdlib.h>
#include <string.h>

#include "viam_carto.h"
#include <stdio.h>
#include <unistd.h>

viam_carto viam_carto_New(const char *sensors) {
  return (viam_carto){.sensors = sensors};
}

// viam_carto_Init takes a viam_carto, and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0 & mutates viam_carto to contain handle to
// initialized carto object
/* #define VIAM_CARTO_INIT_ERR */
int viam_carto_Init(viam_carto *viam_carto, char **errmsg) {
#ifdef VIAM_CARTO_INIT_ERR
  char *err;
  err = "there was an error in init\n";
  *errmsg = err;
  return 1;
#endif
  return 0;
}

// viam_carto_Close takes a viam_carto, and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, frees all resources associated with viam_carto
int viam_carto_Close(viam_carto *viam_carto, char **errmsg) {
  free(viam_carto);
  return 0;
}

int viam_carto_DestroyGetPositionResponse(
    viam_carto_get_position_response *response) {
  free(response);
  return 0;
}

// viam_carto_GetPosition takes a viam_carto, an empty
// viam_carto_get_position_response and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, mutates response to contain response
int viam_carto_GetPosition(const viam_carto *viam_carto,
                           viam_carto_get_position_response *response,
                           char **errmsg) {
  viam_carto_get_position_response res;

  res = (viam_carto_get_position_response){.component_reference =
                                               "some_component_reference",
                                           .x = 0,
                                           .y = 1,
                                           .z = 2,
                                           .o_x = 3,
                                           .o_y = 4,
                                           .o_z = 5,
                                           .theta = 6};
  *response = res;
  return 0;
}

viam_carto_sensor_reading viam_carto_NewSensorReading(char *sensor,
                                                      char *sensor_reading,
                                                      int sensor_reading_len) {
  return (viam_carto_sensor_reading){.sensor_reading_len = sensor_reading_len,
                                     .sensor = sensor,
                                     .sensor_reading = sensor_reading};
}

// viam_carto_WriteSensor takes a viam_carto, a viam_carto_sensor_reading, and
// an empty errmsg
//
// Freeing viam_carto_sensor_reading after calling viam_carto_WriteSensor
// is the caller's responsibility.
//
// NOTE: Blocking!
// If viam_carto is not ready to receive lidar readings this function
// blocks until it is ready to receive.
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, mutates response to contain response
int viam_carto_WriteSensor(const viam_carto *viam_carto,
                           const viam_carto_sensor_reading *sensor_reading,
                           char **errmsg) {

  printf("printing sensor_reading->sensor_reading\n");

  for (int i = 0; i < sensor_reading->sensor_reading_len; i++) {
    printf("%d", sensor_reading->sensor_reading[i]);
  }
  printf("\n");

  sleep(3);
  return 0;
}

viam_carto_ERROR viam_carto_init(viam_carto *vc, const viam_carto_config c,
                                 const viam_carto_algo_config ac,
                                 char **errmsg) {

  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_start(viam_carto *vc, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_stop(viam_carto *vc, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_terminate(viam_carto *vc, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_add_sensor_reading(
    const viam_carto *vc, const viam_carto_sensor_reading *sr, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR
viam_carto_add_sensor_reading_destroy(viam_carto_sensor_reading *sr,
                                      char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_get_position(const viam_carto *vc,
                                         viam_carto_get_position_response *r,
                                         char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};
viam_carto_ERROR
viam_carto_get_position_response_destroy(viam_carto_get_position_response *r,
                                         char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR
viam_carto_get_point_cloud_map(const viam_carto *vc,
                               viam_carto_get_point_cloud_map_response *r,
                               char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR
viam_carto_get_internal_state(const viam_carto *vc,
                              viam_carto_get_internal_state_response *r,
                              char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};

viam_carto_ERROR viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
};
