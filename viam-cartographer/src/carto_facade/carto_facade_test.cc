#include "carto_facade.h"

#include <boost/test/unit_test.hpp>
#include <cstring>
#include <exception>

#include "glog/logging.h"

namespace viam {
namespace carto_facade {
viam_carto_config viam_carto_config_setup() {
    struct viam_carto_config vcc;
    vcc.map_rate_sec = 1;
    vcc.mode = VIAM_CARTO_LOCALIZING;
    vcc.lidar_config = VIAM_CARTO_THREE_D;
    std::string data_dir = "/tmp/some/direcory";
    std::string component_reference = "some component refereance";
    vcc.data_dir = bfromcstr(data_dir.c_str());
    vcc.component_reference = bfromcstr(component_reference.c_str());
    int sensors_len = 5;
    const char *sensors_arr[] = {
        "sensor_1", "sensor_2", "sensor_3", "sensor_4", "sensor_5",
    };
    bstring *sensors = (bstring *)malloc(sizeof(bstring *) * sensors_len);
    BOOST_TEST(sensors != nullptr);
    for (int i = 0; i < sensors_len; i++) {
        sensors[i] = bfromcstr(sensors_arr[i]);
    }
    vcc.sensors_len = sensors_len;
    vcc.sensors = sensors;
    return vcc;
}

void viam_carto_config_teardown(viam_carto_config vcc) {
    BOOST_TEST(bdestroy(vcc.data_dir) == BSTR_OK);
    BOOST_TEST(bdestroy(vcc.component_reference) == BSTR_OK);
    for (int i = 0; i < vcc.sensors_len; i++) {
        BOOST_TEST(bdestroy(vcc.sensors[i]) == BSTR_OK);
    }
    free(vcc.sensors);
}

viam_carto_algo_config viam_carto_algo_config_setup() {
    struct viam_carto_algo_config ac;
    ac.optimize_every_n_nodes = 1;
    ac.num_range_data = 2;
    ac.missing_data_ray_length = 3.1;
    ac.max_range = 4.1;
    ac.min_range = 0.1;
    ac.max_submaps_to_keep = 6;
    ac.fresh_submaps_count = 7;
    ac.min_covered_area = 8.1;
    ac.min_added_submaps_count = 9;
    ac.occupied_space_weight = 10.1;
    ac.translation_weight = 11.1;
    ac.rotation_weight = 12.1;
    return ac;
}

BOOST_AUTO_TEST_SUITE(CartoFacadeCPPAPI)

BOOST_AUTO_TEST_CASE(CartoFacade_lib_init_terminate) {
    viam_carto_lib *lib;
    BOOST_TEST(FLAGS_logtostderr == 0);
    BOOST_TEST(viam_carto_lib_init(nullptr) == VIAM_CARTO_LIB_INVALID);
    BOOST_TEST(FLAGS_logtostderr == 0);
    BOOST_TEST(viam_carto_lib_init(&lib) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(FLAGS_logtostderr == 1);
    BOOST_TEST(lib != nullptr);
    BOOST_TEST(lib->initialized == true);
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(lib == nullptr);
    BOOST_TEST(FLAGS_logtostderr == 0);
}

BOOST_AUTO_TEST_CASE(CartoFacade_init_terminate) {
    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(lib->initialized == true);

    viam_carto *vc;
    struct viam_carto_config vcc = viam_carto_config_setup();
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();
    BOOST_TEST(viam_carto_init(nullptr, lib, vcc, ac) == VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_init(nullptr, nullptr, vcc, ac) ==
               VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_init(&vc, nullptr, vcc, ac) ==
               VIAM_CARTO_LIB_INVALID);
    lib->initialized = false;
    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) ==
               VIAM_CARTO_LIB_NOT_INITIALIZED);
    lib->initialized = true;
    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);

    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_SUCCESS);
    viam_carto_config_teardown(vcc);

    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_demo) {
    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib) == VIAM_CARTO_SUCCESS);

    // Setup
    viam_carto *vc;
    struct viam_carto_config vcc = viam_carto_config_setup();
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);

    // // Start
    BOOST_TEST(viam_carto_start(vc) == VIAM_CARTO_SUCCESS);

    // // GetPosition
    viam_carto_get_position_response pr;
    // // Test
    BOOST_TEST(viam_carto_get_position(vc, &pr) == VIAM_CARTO_SUCCESS);

    BOOST_TEST(pr.x == 100);
    BOOST_TEST(pr.y == 200);
    BOOST_TEST(pr.z == 300);
    BOOST_TEST(pr.o_x == 400);
    BOOST_TEST(pr.o_y == 500);
    BOOST_TEST(pr.o_z == 600);
    BOOST_TEST(pr.imag == 700);
    BOOST_TEST(pr.jmag == 800);
    BOOST_TEST(pr.kmag == 900);
    BOOST_TEST(pr.theta == 1000);
    BOOST_TEST(pr.real == 1100);
    bstring cr = bfromcstr("C++ component reference");
    BOOST_TEST(biseq(pr.component_reference, cr) == true);

    // GetPosition Teardown
    BOOST_TEST(bdestroy(cr) == BSTR_OK);
    BOOST_TEST(viam_carto_get_position_response_destroy(&pr) ==
               VIAM_CARTO_SUCCESS);

    // AddSensorReading
    viam_carto_sensor_reading sr;
    BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) == VIAM_CARTO_SUCCESS);

    BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
               VIAM_CARTO_SUCCESS);

    // GetPointCloudMap
    viam_carto_get_point_cloud_map_response mr;
    BOOST_TEST(viam_carto_get_point_cloud_map(vc, &mr) == VIAM_CARTO_SUCCESS);

    BOOST_TEST(viam_carto_get_point_cloud_map_response_destroy(&mr) ==
               VIAM_CARTO_SUCCESS);

    // GetInternalState
    viam_carto_get_internal_state_response isr;
    BOOST_TEST(viam_carto_get_internal_state(vc, &isr) == VIAM_CARTO_SUCCESS);

    BOOST_TEST(viam_carto_get_internal_state_response_destroy(&isr) ==
               VIAM_CARTO_SUCCESS);

    // Stop
    BOOST_TEST(viam_carto_stop(vc) == VIAM_CARTO_SUCCESS);

    // Terminate
    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_SUCCESS);
    viam_carto_config_teardown(vcc);

    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_config) {
    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib) == VIAM_CARTO_SUCCESS);

    struct viam_carto_config vcc = viam_carto_config_setup();

    struct config c = viam::carto_facade::from_viam_carto_config(vcc);

    BOOST_TEST(c.component_reference == "some component refereance");
    BOOST_TEST(c.data_dir == "/tmp/some/direcory");
    BOOST_TEST(c.lidar_config == VIAM_CARTO_THREE_D);
    BOOST_TEST(c.map_rate_sec == 1);
    BOOST_TEST(c.mode == VIAM_CARTO_LOCALIZING);
    BOOST_TEST(c.sensors.size() == 5);
    BOOST_TEST(c.sensors[0] == "sensor_1");
    BOOST_TEST(c.sensors[1] == "sensor_2");
    BOOST_TEST(c.sensors[2] == "sensor_3");
    BOOST_TEST(c.sensors[3] == "sensor_4");
    BOOST_TEST(c.sensors[4] == "sensor_5");

    viam_carto_config_teardown(vcc);

    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace carto_facade
}  // namespace viam
