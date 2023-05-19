#include <stdlib.h>
#include <string.h>

#include "viam_carto.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int viam_carto_init(viam_carto **ppVC, const viam_carto_config c,
                    const viam_carto_algo_config ac, char **errmsg) {
  UNUSED(ac);
  if (ppVC == NULL) {
    *errmsg = "viam_carto pointer should not be NULL";
    return VIAM_CARTO_VC_INVALID;
  }

  viam_carto *vc = malloc(sizeof(viam_carto));
  vc->sensors_len = c.sensors_len;
  int i = 0;
  for (i = 0; i < c.sensors_len; i++) {
    char *sensor;
    // TODO: FIX THIS: From the man pages
    // SECURITY CONSIDERATIONS
    // The strcpy() function is easily misused in a manner which enables
    // malicious users to arbitrarily change a running program's functionality
    // through a buffer overflow attack.
    strcpy(sensor, c.sensors[i]);
    c.sensors[i] = sensor;
  }

  *ppVC = vc;

  return VIAM_CARTO_SUCCESS;
};

int viam_carto_terminate(viam_carto **ppVC, char **errmsg) {
  UNUSED(errmsg);
  // fix this
  vc->sensors_len = ppVC.c.sensors_len;
  int i = 0;
  for (i = 0; i < c.sensors_len; i++) {
    char *sensor;
    // TODO: FIX THIS: From the man pages
    // SECURITY CONSIDERATIONS
    // The strcpy() function is easily misused in a manner which enables
    // malicious users to arbitrarily change a running program's functionality
    // through a buffer overflow attack.
    strcpy(sensor, c.sensors[i]);
    c.sensors[i] = sensor;
  }
  free(*ppVC);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_start(viam_carto **ppVC, char **errmsg) {
  UNUSED(ppVC);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_stop(viam_carto **ppVC, char **errmsg) {
  return VIAM_CARTO_SUCCESS;
  UNUSED(ppVC);
  UNUSED(errmsg);
};

int viam_carto_add_sensor_reading(const viam_carto **ppVC,
                                  const viam_carto_sensor_reading *sr,
                                  char **errmsg) {
  UNUSED(ppVC);
  UNUSED(sr);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_add_sensor_reading_destroy(viam_carto_sensor_reading *sr,
                                          char **errmsg) {
  UNUSED(sr);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_get_position(const viam_carto **ppVC,
                            viam_carto_get_position_response *r,
                            char **errmsg) {
  UNUSED(ppVC);
  UNUSED(r);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};
int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r, char **errmsg) {
  UNUSED(r);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_get_point_cloud_map(const viam_carto **ppVC,
                                   viam_carto_get_point_cloud_map_response *r,
                                   char **errmsg) {
  UNUSED(ppVC);
  UNUSED(r);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r, char **errmsg) {
  UNUSED(r);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_get_internal_state(const viam_carto **ppVC,
                                  viam_carto_get_internal_state_response *r,
                                  char **errmsg) {
  UNUSED(ppVC);
  UNUSED(r);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r, char **errmsg) {
  UNUSED(r);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};
