#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <stdio.h>
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
  int initialized_flag; // Currently used to simulate initialization of carto_obj 
  void *carto_obj;
} viam_carto;

// GetPositionResponse https://github.com/viamrobotics/api/blob/main/proto/viam/service/slam/v1/slam.proto#L51
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
  // TODO: Need to also return quat information as the spaital math exists only on the go side
} viam_carto_get_position_response;

typedef struct viam_carto_sensor_reading {
  int sensor_reading_len;
  char *sensor;
  char *sensor_reading;
} viam_carto_sensor_reading;

// viam_carto_New takes a viam_carto, and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0 & mutates viam_carto to contain handle to
// initialized carto object
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
  viam_carto->initialized_flag = 5;
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

// viam_carto_GetPosition takes a viam_carto, an empty viam_carto_get_position_response and an empty
// errmsg
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

// viam_carto_WriteSensor takes a viam_carto, a viam_carto_sensor_reading, and an empty errmsg
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
