// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

namespace viam {
namespace carto_facade {
int CartoFacade::GetPosition(viam_carto_get_position_response *r) { return 0; };

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
