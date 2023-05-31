// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

namespace viam {
namespace carto_facade {
int CartoFacade::GetPosition(viam_carto_get_position_response *r) { 
    bstring cr = bfromcstr("C++ component reference");
    r->x = 100;
    r->y = 200;
    r->z = 300;
    r->o_x = 400;
    r->o_y = 500;
    r->o_z = 600;
    r->imag = 700;
    r->jmag = 800;
    r->kmag = 900;
    r->theta = 1000;
    r->real = 1100;
    r->component_reference = cr;
    return 0;
};

int CartoFacade::GetPointCloudMap(viam_carto_get_point_cloud_map_response *r) {
    return 0;
};
int CartoFacade::GetInternalState(viam_carto_get_internal_state_response *r) {
    return 0;
};

int CartoFacade::Start() { return 0; };

int CartoFacade::Stop() { return 0; };

int CartoFacade::AddSensorReading(viam_carto_sensor_reading *sr) { return 0; };
}  // namespace carto_facade
}  // namespace viam

extern int viam_carto_init(const viam_carto **vc, const viam_carto_config c,
                           const viam_carto_algo_config ac, char **errmsg) {
    return 0;
};

extern int viam_carto_start(const viam_carto **vc, char **errmsg) { return 0; };

extern int viam_carto_stop(const viam_carto **vc, char **errmsg) { return 0; };

extern int viam_carto_terminate(const viam_carto **vc, char **errmsg) {
    return 0;
};

extern int viam_carto_add_sensor_reading(const viam_carto **vc,
                                         const viam_carto_sensor_reading *sr,
                                         char **errmsg) {
    return 0;
};

extern int viam_carto_add_sensor_reading_destroy(viam_carto_sensor_reading *sr,
                                                 char **errmsg) {
    return 0;
};

extern int viam_carto_get_position(const viam_carto **vc,
                                   viam_carto_get_position_response *r,
                                   char **errmsg) {
    return 0;
};

extern int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r, char **errmsg) {
    return 0;
};

extern int viam_carto_get_point_cloud_map(
    const viam_carto **vc, viam_carto_get_point_cloud_map_response *r,
    char **errmsg) {
    return 0;
};

extern int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r, char **errmsg) {
    return 0;
};

extern int viam_carto_get_internal_state(
    const viam_carto **vc, viam_carto_get_internal_state_response *r,
    char **errmsg) {
    return 0;
};

extern int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r, char **errmsg) {
    return 0;
};
