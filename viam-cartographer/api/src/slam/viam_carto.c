#include <stdlib.h>

#include <string.h>

#include "bstrlib.h"
#include "carto_facade.h"
#include "viam_carto.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int viam_carto_init(const viam_carto **ppVC, const viam_carto_config c,
                    const viam_carto_algo_config ac, char **errmsg) {
  UNUSED(ac);
  if (ppVC == NULL) {
    *errmsg = "viam_carto pointer should not be NULL";
    return VIAM_CARTO_VC_INVALID;
  }

  // allocate viam_carto struct
  viam_carto *vc = (viam_carto *)malloc(sizeof(viam_carto));

  // allocate sensors list
  bstring *sensors = (bstring *)malloc(sizeof(bstring *) * c.sensors_len);
  if (sensors == NULL) {
    return VIAM_CARTO_OUT_OF_MEMORY;
  }

  // copy sensors to sensors list
  int i = 0;
  for (i = 0; i < c.sensors_len; i++) {
    sensors[i] = bstrcpy(c.sensors[i]);
  }
  // set sensors list
  vc->sensors = sensors;
  vc->sensors_len = c.sensors_len;
  vc->carto_obj = carto_facade_new();

  // point to newly created viam_carto struct
  *ppVC = vc;

  return VIAM_CARTO_SUCCESS;
};

int viam_carto_terminate(const viam_carto **ppVC, char **errmsg) {
  UNUSED(errmsg);
  int i = 0;
  int rc = BSTR_OK;
  /* (*ppVCk-->sensors ; */
  int return_code = VIAM_CARTO_SUCCESS;
  for (i = 0; i < (*ppVC)->sensors_len; i++) {
    rc = bdestroy((*ppVC)->sensors[i]);
    if (rc != BSTR_OK) {
      // TODO: Write error messages
      return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
  }
  free((*ppVC)->sensors);
  // TODO: Error handling
  carto_facade_delete((*ppVC)->carto_obj);
  free((viam_carto *)*ppVC);
  return return_code;
};

int viam_carto_start(const viam_carto **ppVC, char **errmsg) {
  UNUSED(ppVC);
  UNUSED(errmsg);
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_stop(const viam_carto **ppVC, char **errmsg) {
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
  UNUSED(errmsg);
  viam_carto_get_position_response gpr =
      carto_facade_get_position((*ppVC)->carto_obj);
  *r = gpr;
  return VIAM_CARTO_SUCCESS;
};

int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r, char **errmsg) {
  UNUSED(errmsg);
  int return_code = VIAM_CARTO_SUCCESS;
  int rc = BSTR_OK;
  rc = bdestroy(r->component_reference);
  if (rc != BSTR_OK) {
    // TODO: Write error messages
    return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
  }
  r->component_reference = NULL;
  return VIAM_CARTO_SUCCESS;
  return return_code;
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
