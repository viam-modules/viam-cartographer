// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

#include "glog/logging.h"

namespace viam {
namespace carto_facade {

std::string std_string_from_bstring(bstring b_str) {
    int len = blength(b_str);
    char *tmp = bstr2cstr(b_str, 0);
    std::string std_str(tmp, len);
    bcstrfree(tmp);
    return std_str;
}

config from_viam_carto_config(viam_carto_config vcc) {
    struct config c;
    for (int i = 0; i < vcc.sensors_len; i++) {
        c.sensors.push_back(std_string_from_bstring(vcc.sensors[i]));
    }

    c.data_dir = std_string_from_bstring(vcc.data_dir);
    c.component_reference = std_string_from_bstring(vcc.component_reference);
    c.map_rate_sec = vcc.map_rate_sec;
    c.mode = vcc.mode;
    c.lidar_config = vcc.lidar_config;
    return c;
};

CartoFacade::CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                         const viam_carto_algo_config ac) {
    lib = pVCL;
    config = from_viam_carto_config(c);
    algo_config = ac;
};

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
    return VIAM_CARTO_SUCCESS;
};

int CartoFacade::GetPointCloudMap(viam_carto_get_point_cloud_map_response *r) {
    return VIAM_CARTO_SUCCESS;
};
int CartoFacade::GetInternalState(viam_carto_get_internal_state_response *r) {
    return VIAM_CARTO_SUCCESS;
};

int CartoFacade::Start() { return VIAM_CARTO_SUCCESS; };

int CartoFacade::Stop() { return VIAM_CARTO_SUCCESS; };

int CartoFacade::AddSensorReading(viam_carto_sensor_reading *sr) {
    return VIAM_CARTO_SUCCESS;
};
}  // namespace carto_facade
}  // namespace viam

extern int viam_carto_lib_init(viam_carto_lib **ppVCL, int minloglevel,
                               int verbose) {
    if (ppVCL == nullptr) {
        return VIAM_CARTO_LIB_INVALID;
    }
    if (!((sizeof(float) == 4) && (CHAR_BIT == 8) && (sizeof(int) == 4))) {
        return VIAM_CARTO_LIB_PLATFORM_INVALID;
    }
    viam_carto_lib *vcl = (viam_carto_lib *)malloc(sizeof(viam_carto_lib));
    if (vcl == nullptr) {
        return VIAM_CARTO_OUT_OF_MEMORY;
    }
    google::InitGoogleLogging("cartographer");
    FLAGS_logtostderr = 1;
    FLAGS_minloglevel = minloglevel;
    FLAGS_v = verbose;
    vcl->minloglevel = minloglevel;
    vcl->verbose = verbose;

    *ppVCL = vcl;

    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_lib_terminate(viam_carto_lib **ppVCL) {
    FLAGS_logtostderr = 0;
    FLAGS_minloglevel = 0;
    FLAGS_v = 0;
    google::ShutdownGoogleLogging();
    free(*ppVCL);
    *ppVCL = nullptr;
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_init(viam_carto **ppVC, viam_carto_lib *pVCL,
                           const viam_carto_config c,
                           const viam_carto_algo_config ac) {
    if (ppVC == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (pVCL == nullptr) {
        return VIAM_CARTO_LIB_INVALID;
    }

    if (!pVCL->initialized) {
        return VIAM_CARTO_LIB_NOT_INITIALIZED;
    }

    // allocate viam_carto struct
    viam_carto *vc = (viam_carto *)malloc(sizeof(viam_carto));
    if (vc == nullptr) {
        return VIAM_CARTO_OUT_OF_MEMORY;
    }

    vc->carto_obj = new viam::carto_facade::CartoFacade(pVCL, c, ac);

    // point to newly created viam_carto struct
    *ppVC = vc;
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_start(viam_carto *vc) { return VIAM_CARTO_SUCCESS; };

extern int viam_carto_stop(viam_carto *vc) { return VIAM_CARTO_SUCCESS; };

extern int viam_carto_terminate(viam_carto **ppVC) {
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>((*ppVC)->carto_obj);
    delete cf;
    free((viam_carto *)*ppVC);
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_sensor_reading(viam_carto *vc,
                                         const viam_carto_sensor_reading *sr) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_sensor_reading_destroy(
    viam_carto_sensor_reading *sr) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_position(viam_carto *vc,
                                   viam_carto_get_position_response *r) {
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>((vc)->carto_obj);
    cf->GetPosition(r);
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r) {
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;
    rc = bdestroy(r->component_reference);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    r->component_reference = nullptr;
    return return_code;
};

extern int viam_carto_get_point_cloud_map(
    viam_carto *vc, viam_carto_get_point_cloud_map_response *r) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_internal_state(
    viam_carto *vc, viam_carto_get_internal_state_response *r) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r) {
    return VIAM_CARTO_SUCCESS;
};
