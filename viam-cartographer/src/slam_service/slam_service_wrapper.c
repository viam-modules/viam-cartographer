#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

void Hello() { printf("FROM WITHIN C PROGRAM: Hello world\n"); }

int sum(int a, int b) { return a + b; }

void reverse(char *x, int begin, int end) {
  char c;
  if (begin >= end)
    return;
  c = *(x + begin);
  *(x + begin) = *(x + end);
  *(x + end) = c;

  reverse(x, ++begin, --end);
  printf("FROM WITHIN C PROGRAM:\n");
  puts(x);
}

// https://valgrind.org/docs/manual/quick-start.html
void trip_valgrind(void) {
  int *x = malloc(10 * sizeof(int)); // problem 1: heap block overrun
  x[10] = 0;                         // problem 2: memory leak -- x not freed
}

// https://github.com/google/sanitizers/wiki/AddressSanitizerExampleUseAfterReturn
int *ptr;
__attribute__((noinline)) void FunctionThatEscapesLocalObject() {
  int local[100];
  ptr = &local[0];
}

int trip_address_sanitizer() {
  int x = 1;
  FunctionThatEscapesLocalObject();
  return ptr[x];
}

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
 */

typedef struct viam_carto viam_carto;

struct viam_carto {
  const char *sensors;
};

typedef struct viam_carto_get_position_response
    viam_carto_get_position_response;
struct viam_carto_get_position_response {
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
};

typedef struct viam_carto_sensor_reading viam_carto_sensor_reading;
struct viam_carto_sensor_reading {
  const int sensor_reading_len;
  const char *sensor;
  const char *sensor_reading;
};

// Takes a viam_carto, and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0 & mutates viam_carto to contain handle to
// initialized carto object
viam_carto viam_carto_New(const char *sensors) {
  return (viam_carto){.sensors = sensors};
}

// Takes a viam_carto, and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0 & mutates viam_carto to contain handle to
// initialized carto object
int viam_carto_Init(viam_carto **viam_carto, char **errmsg) { return 0; }

// Takes a viam_carto, and an empty errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, frees all resources associated with viam_carto
int viam_carto_Close(viam_carto **viam_carto, char **errmsg) { return 0; }

// Takes a viam_carto, an empty viam_carto_get_position_response and an empty
// errmsg
//
// On error: Returns a non 0 error code and mutates
// errmsg with a string that contains an informative error message
//
// On success: Returns 0, mutates response to contain response
int viam_carto_GetPosition(const viam_carto *viam_carto,
                           viam_carto_get_position_response **response,
                           char **errmsg) {
#ifdef VIAM_CARTO_GETPOSITION_ERR
  char *err;
  err = "there was an error";
  *errmsg = err;
  return 1
#endif
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
  **response = res;
  return 0;
}

// Takes a viam_carto, a viam_carto_sensor_reading, and an empty errmsg
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
int viam_carto_WriteSensor(const viam_carto **viam_carto,
                           const viam_carto_sensor_reading *sensor_reading,
                           char **errmsg) {

  sleep(3);
  return 0;
}
