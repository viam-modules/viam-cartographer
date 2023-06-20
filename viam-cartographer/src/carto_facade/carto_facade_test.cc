#include "carto_facade.h"

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <cstring>
#include <exception>
#include <filesystem>

#include "glog/logging.h"

namespace tt = boost::test_tools;
namespace fs = std::filesystem;
namespace bfs = boost::filesystem;

namespace viam {
namespace carto_facade {
viam_carto_config viam_carto_config_setup(
    int map_rate_sec, viam_carto_LIDAR_CONFIG lidar_config,
    std::string data_dir, std::string component_reference,
    std::vector<std::string> sensors_vec) {
    struct viam_carto_config vcc;
    vcc.map_rate_sec = map_rate_sec;
    vcc.lidar_config = lidar_config;
    vcc.data_dir = bfromcstr(data_dir.c_str());
    vcc.component_reference = bfromcstr(component_reference.c_str());
    bstring *sensors =
        (bstring *)malloc(sizeof(bstring *) * sensors_vec.size());
    BOOST_TEST(sensors != nullptr);
    int i = 0;
    for (auto &&s : sensors_vec) {
        sensors[i] = bfromcstr(s.c_str());
        i++;
    }
    vcc.sensors_len = sensors_vec.size();
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
    ac.optimize_on_start = false;
    ac.optimize_every_n_nodes = 3;
    ac.num_range_data = 100;
    ac.missing_data_ray_length = 25.0;
    ac.max_range = 25.0;
    ac.min_range = 0.2;
    ac.max_submaps_to_keep = 3;
    ac.fresh_submaps_count = 3;
    ac.min_covered_area = 1.0;
    ac.min_added_submaps_count = 1;
    ac.occupied_space_weight = 20.0;
    ac.translation_weight = 10.0;
    ac.rotation_weight = 1.0;
    return ac;
}

BOOST_AUTO_TEST_SUITE(CartoFacadeCPPAPI, *boost::unit_test::disabled());

BOOST_AUTO_TEST_CASE(CartoFacade_lib_init_terminate) {
    viam_carto_lib *lib;
    BOOST_TEST(FLAGS_logtostderr == 0);
    BOOST_TEST(viam_carto_lib_init(nullptr, 0, 0) == VIAM_CARTO_LIB_INVALID);

    BOOST_TEST(FLAGS_logtostderr == 0);
    BOOST_TEST(FLAGS_v == 0);
    BOOST_TEST(FLAGS_minloglevel == 0);
    BOOST_TEST(viam_carto_lib_init(&lib, 2, 3) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(lib != nullptr);
    BOOST_TEST(lib->minloglevel == 2);
    BOOST_TEST(lib->verbose == 3);
    // begin global side effects
    BOOST_TEST(FLAGS_logtostderr == 1);
    BOOST_TEST(FLAGS_minloglevel == 2);
    BOOST_TEST(FLAGS_v == 3);
    // end global side effects
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(lib == nullptr);
    BOOST_TEST(FLAGS_logtostderr == 0);
    BOOST_TEST(FLAGS_v == 0);
    BOOST_TEST(FLAGS_minloglevel == 0);
}

BOOST_AUTO_TEST_CASE(CartoFacade_init_validate) {
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    viam_carto *vc;
    std::vector<std::string> empty_sensors_vec;
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());

    struct viam_carto_config vcc_no_sensors =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),
                                "some component refereance", empty_sensors_vec);
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_no_sensors, ac) ==
               VIAM_CARTO_SENSORS_LIST_EMPTY);

    std::vector<std::string> sensors_vec;
    sensors_vec.push_back("sensor_1");
    sensors_vec.push_back("sensor_2");
    sensors_vec.push_back("sensor_3");
    sensors_vec.push_back("sensor_4");
    sensors_vec.push_back("sensor_5");
    struct viam_carto_config vcc_empty_data_dir = viam_carto_config_setup(
        1, VIAM_CARTO_THREE_D, "", "some component refereance", sensors_vec);

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_empty_data_dir, ac) ==
               VIAM_CARTO_DATA_DIR_NOT_PROVIDED);

    struct viam_carto_config vcc_empty_component_ref = viam_carto_config_setup(
        1, VIAM_CARTO_THREE_D, tmp_dir.string(), "", sensors_vec);

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_empty_component_ref, ac) ==
               VIAM_CARTO_COMPONENT_REFERENCE_INVALID);

    struct viam_carto_config vcc_invalid_map_rate_sec =
        viam_carto_config_setup(-1, VIAM_CARTO_THREE_D, tmp_dir.string(),
                                "some component refereance", sensors_vec);

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_invalid_map_rate_sec, ac) ==
               VIAM_CARTO_MAP_RATE_SEC_INVALID);

    struct viam_carto_config vcc_invalid_lidar_config = viam_carto_config_setup(
        1, static_cast<viam_carto_LIDAR_CONFIG>(-1), tmp_dir.string(),
        "some component refereance", sensors_vec);

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_invalid_lidar_config, ac) ==
               VIAM_CARTO_LIDAR_CONFIG_INVALID);

    fs::path deprecated_path = tmp_dir / fs::path(bfs::unique_path().string());
    fs::create_directories(deprecated_path.string() + "/data");

    struct viam_carto_config vcc_deprecated_path =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, deprecated_path.string(),
                                "some component refereance", sensors_vec);
    BOOST_TEST(viam_carto_init(&vc, lib, vcc_deprecated_path, ac) ==
               VIAM_CARTO_DATA_DIR_INVALID_DEPRECATED_STRUCTURE);

    fs::path invalid_path = tmp_dir / fs::path(bfs::unique_path().string()) /
                            fs::path(bfs::unique_path().string());

    struct viam_carto_config vcc_invalid_path =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, invalid_path.string(),

                                "some component refereance", sensors_vec);
    BOOST_TEST(viam_carto_init(&vc, lib, vcc_invalid_path, ac) ==
               VIAM_CARTO_DATA_DIR_FILE_SYSTEM_ERROR);

    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),
                                "some component refereance", sensors_vec);

    BOOST_TEST(viam_carto_init(nullptr, lib, vcc, ac) == VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_init(nullptr, nullptr, vcc, ac) ==
               VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_init(&vc, nullptr, vcc, ac) ==
               VIAM_CARTO_LIB_INVALID);
    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_SUCCESS);

    viam_carto_config_teardown(vcc_no_sensors);
    viam_carto_config_teardown(vcc_empty_data_dir);
    viam_carto_config_teardown(vcc_empty_component_ref);
    viam_carto_config_teardown(vcc_invalid_map_rate_sec);
    viam_carto_config_teardown(vcc_invalid_lidar_config);
    viam_carto_config_teardown(vcc_deprecated_path);
    viam_carto_config_teardown(vcc_invalid_path);
    viam_carto_config_teardown(vcc);
    // TODO: Move all suite level setup & teardown to boost test hook
    fs::remove_all(tmp_dir);

    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_init_derive_action_mode) {
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    std::vector<std::string> empty_sensors_vec;
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    std::vector<std::string> sensors_vec;
    sensors_vec.push_back("sensor_1");
    sensors_vec.push_back("sensor_2");
    sensors_vec.push_back("sensor_3");
    sensors_vec.push_back("sensor_4");
    sensors_vec.push_back("sensor_5");
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    fs::create_directory(tmp_dir);
    {
        // mapping
        viam_carto *vc1;
        auto mapping_dir = tmp_dir / fs::path("mapping_dir");
        struct viam_carto_config vcc_mapping =
            viam_carto_config_setup(1, VIAM_CARTO_THREE_D, mapping_dir.string(),
                                    "some component refereance", sensors_vec);
        BOOST_TEST(viam_carto_init(&vc1, lib, vcc_mapping, ac) ==
                   VIAM_CARTO_SUCCESS);
        viam::carto_facade::CartoFacade *cf1 =
            static_cast<viam::carto_facade::CartoFacade *>(vc1->carto_obj);
        BOOST_TEST(cf1->action_mode == ActionMode::MAPPING);
        BOOST_TEST(cf1->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf1->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf1->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tt::tolerance(0.001));
        BOOST_TEST(cf1->map_builder.GetMaxRange() == ac.max_range,
                   tt::tolerance(0.001));
        BOOST_TEST(cf1->map_builder.GetMinRange() == ac.min_range,
                   tt::tolerance(0.001));
        BOOST_TEST(cf1->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tt::tolerance(0.001));
        BOOST_TEST(
            cf1->map_builder.GetTranslationWeight() == ac.translation_weight,
            tt::tolerance(0.001));
        BOOST_TEST(cf1->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tt::tolerance(0.001));
        // END TEST
        BOOST_TEST(viam_carto_terminate(&vc1) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_mapping);
    }

    auto updating_dir = tmp_dir / fs::path("updating_dir");
    // updating setup
    {
        auto internal_state_dir = updating_dir / fs::path("internal_state");
        fs::create_directories(internal_state_dir);
        // we need to copy a valid internal state when we boot in updating mode
        // otherwise cartographer terminates the entire os process
        // see: https://viam.atlassian.net/browse/RSDK-3553
        auto internal_state_artifact_source =
            fs::current_path() /
            fs::path(
                ".artifact/data/viam-cartographer/outputs/viam-office-02-22-3/"
                "internal_state/internal_state_0.pbstream");

        VLOG(1) << "internal_state_artifact_source: "
                << internal_state_artifact_source;
        VLOG(1) << "exists(internal_state_artifact_source): "
                << exists(internal_state_artifact_source);
        VLOG(1) << "internal_state_dir: " << internal_state_dir;
        VLOG(1) << "exists(internal_state_dir): " << exists(internal_state_dir);
        auto internal_state_artifact_target =
            internal_state_dir /
            fs::path("map_data_2022-02-11T01:44:53.1903Z.pbstream");
        fs::copy_file(internal_state_artifact_source,
                      internal_state_artifact_target);
    }

    {
        // updating
        viam_carto *vc2;

        struct viam_carto_config vcc_updating = viam_carto_config_setup(
            1, VIAM_CARTO_THREE_D, updating_dir.string(),
            "some component refereance", sensors_vec);
        BOOST_TEST(viam_carto_init(&vc2, lib, vcc_updating, ac) ==
                   VIAM_CARTO_SUCCESS);
        viam::carto_facade::CartoFacade *cf2 =
            static_cast<viam::carto_facade::CartoFacade *>(vc2->carto_obj);
        BOOST_TEST(cf2->action_mode == ActionMode::UPDATING);
        BOOST_TEST(cf2->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf2->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf2->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tt::tolerance(0.001));
        BOOST_TEST(cf2->map_builder.GetMaxRange() == ac.max_range,
                   tt::tolerance(0.001));
        BOOST_TEST(cf2->map_builder.GetMinRange() == ac.min_range,
                   tt::tolerance(0.001));
        BOOST_TEST(cf2->map_builder.GetFreshSubmapsCount() ==
                   ac.fresh_submaps_count);
        BOOST_TEST(cf2->map_builder.GetMinCoveredArea() == ac.min_covered_area,
                   tt::tolerance(0.001));
        BOOST_TEST(cf2->map_builder.GetMinAddedSubmapsCount() ==
                   ac.min_added_submaps_count);
        BOOST_TEST(cf2->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tt::tolerance(0.001));
        BOOST_TEST(
            cf2->map_builder.GetTranslationWeight() == ac.translation_weight,
            tt::tolerance(0.001));
        BOOST_TEST(cf2->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tt::tolerance(0.001));
        BOOST_TEST(viam_carto_terminate(&vc2) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_updating);
    }
    struct viam_carto_algo_config ac_optimize_on_start =
        viam_carto_algo_config_setup();
    ac_optimize_on_start.optimize_on_start = true;

    {
        // updating optimize_on_start
        viam_carto *vc3;
        struct viam_carto_config vcc_updating = viam_carto_config_setup(
            1, VIAM_CARTO_THREE_D, updating_dir.string(),
            "some component refereance", sensors_vec);

        BOOST_TEST(viam_carto_init(&vc3, lib, vcc_updating,
                                   ac_optimize_on_start) == VIAM_CARTO_SUCCESS);
        viam::carto_facade::CartoFacade *cf2 =
            static_cast<viam::carto_facade::CartoFacade *>(vc3->carto_obj);
        BOOST_TEST(cf2->action_mode == ActionMode::UPDATING);
        BOOST_TEST(viam_carto_terminate(&vc3) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_updating);
    }

    {
        // localizing
        viam_carto *vc4;
        struct viam_carto_config vcc_localizing = viam_carto_config_setup(
            0, VIAM_CARTO_THREE_D, updating_dir.string(),
            "some component refereance", sensors_vec);
        BOOST_TEST(viam_carto_init(&vc4, lib, vcc_localizing, ac) ==
                   VIAM_CARTO_SUCCESS);
        viam::carto_facade::CartoFacade *cf3 =
            static_cast<viam::carto_facade::CartoFacade *>(vc4->carto_obj);
        BOOST_TEST(cf3->action_mode == ActionMode::LOCALIZING);
        BOOST_TEST(cf3->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf3->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf3->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tt::tolerance(0.001));
        BOOST_TEST(cf3->map_builder.GetMaxRange() == ac.max_range,
                   tt::tolerance(0.001));
        BOOST_TEST(cf3->map_builder.GetMinRange() == ac.min_range,
                   tt::tolerance(0.001));
        BOOST_TEST(cf3->map_builder.GetMaxSubmapsToKeep() ==
                   ac.max_submaps_to_keep);
        BOOST_TEST(cf3->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tt::tolerance(0.001));
        BOOST_TEST(
            cf3->map_builder.GetTranslationWeight() == ac.translation_weight,
            tt::tolerance(0.001));
        BOOST_TEST(cf3->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tt::tolerance(0.001));
        BOOST_TEST(viam_carto_terminate(&vc4) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_localizing);
    }

    {
        // localizing optimize_on_start
        viam_carto *vc5;
        struct viam_carto_config vcc_localizing = viam_carto_config_setup(
            0, VIAM_CARTO_THREE_D, updating_dir.string(),
            "some component refereance", sensors_vec);
        BOOST_TEST(viam_carto_init(&vc5, lib, vcc_localizing,
                                   ac_optimize_on_start) == VIAM_CARTO_SUCCESS);
        viam::carto_facade::CartoFacade *cf3 =
            static_cast<viam::carto_facade::CartoFacade *>(vc5->carto_obj);
        BOOST_TEST(cf3->action_mode == ActionMode::LOCALIZING);
        BOOST_TEST(viam_carto_terminate(&vc5) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_localizing);
    }

    {
        // invalid
        auto empty_dir = tmp_dir / fs::path(bfs::unique_path().string());
        ;
        viam_carto *vc6;
        struct viam_carto_config vcc_invalid =
            viam_carto_config_setup(0, VIAM_CARTO_THREE_D, empty_dir.string(),
                                    "some component refereance", sensors_vec);
        BOOST_TEST(viam_carto_init(&vc6, lib, vcc_invalid, ac) ==
                   VIAM_CARTO_SLAM_MODE_INVALID);
        viam_carto_config_teardown(vcc_invalid);
    }

    // TODO: Move all suite level setup & teardown to boost test hook
    fs::remove_all(tmp_dir);

    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_init_terminate) {
    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    viam_carto *vc;
    std::vector<std::string> sensors_vec;
    sensors_vec.push_back("sensor_1");
    sensors_vec.push_back("sensor_2");
    sensors_vec.push_back("sensor_3");
    sensors_vec.push_back("sensor_4");
    sensors_vec.push_back("sensor_5");
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),
                                "some component refereance", sensors_vec);
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();
    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
    BOOST_TEST((cf->lib) == lib);
    BOOST_TEST((cf->algo_config.optimize_on_start) == false);
    BOOST_TEST((cf->algo_config.optimize_every_n_nodes) == 3);
    BOOST_TEST((cf->algo_config.num_range_data) == 100);
    BOOST_TEST((cf->algo_config.missing_data_ray_length) == 25,
               tt::tolerance(0.001));
    BOOST_TEST((cf->algo_config.max_range) == 25, tt::tolerance(0.001));
    BOOST_TEST((cf->algo_config.min_range) == 0.2, tt::tolerance(0.001));
    BOOST_TEST((cf->algo_config.max_submaps_to_keep) == 3);
    BOOST_TEST((cf->algo_config.fresh_submaps_count) == 3);
    BOOST_TEST((cf->algo_config.min_covered_area) == 1, tt::tolerance(0.001));
    BOOST_TEST((cf->algo_config.min_added_submaps_count) == 1);
    BOOST_TEST((cf->algo_config.occupied_space_weight) == 20,
               tt::tolerance(0.001));
    BOOST_TEST((cf->algo_config.translation_weight) == 10,
               tt::tolerance(0.001));
    BOOST_TEST((cf->algo_config.rotation_weight) == 1, tt::tolerance(0.001));
    auto path_to_internal_state = tmp_dir / fs::path("internal_state");

    BOOST_TEST((cf->path_to_internal_state) == path_to_internal_state.string());
    BOOST_TEST((cf->started) == false);
    BOOST_TEST((cf->config.sensors) == sensors_vec);
    BOOST_TEST((cf->config.map_rate_sec).count() == 1);
    BOOST_TEST((cf->config.data_dir) == tmp_dir.string());
    BOOST_TEST((cf->config.component_reference) == "some component refereance");
    BOOST_TEST((cf->config.lidar_config) == VIAM_CARTO_THREE_D);

    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_SUCCESS);
    viam_carto_config_teardown(vcc);
    fs::remove_all(tmp_dir);

    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_demo) {
    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    // Setup
    viam_carto *vc;
    std::vector<std::string> sensors_vec;
    sensors_vec.push_back("sensor_1");
    sensors_vec.push_back("sensor_2");
    sensors_vec.push_back("sensor_3");
    sensors_vec.push_back("sensor_4");
    sensors_vec.push_back("sensor_5");
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),

                                "some component refereance", sensors_vec);
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
    sr.sensor = bfromcstr("my sensor");
    sr.sensor_reading = bfromcstr("some invalid reading");
    sr.sensor_reading_time_unix_micro = 10000;
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
    fs::remove_all(tmp_dir);

    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_config) {
    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    std::vector<std::string> sensors_vec;
    sensors_vec.push_back("sensor_1");
    sensors_vec.push_back("sensor_2");
    sensors_vec.push_back("sensor_3");
    sensors_vec.push_back("sensor_4");
    sensors_vec.push_back("sensor_5");
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),

                                "some component refereance", sensors_vec);

    struct config c = viam::carto_facade::from_viam_carto_config(vcc);

    BOOST_TEST(c.component_reference == "some component refereance");
    BOOST_TEST(c.data_dir == tmp_dir.string());
    BOOST_TEST(c.lidar_config == VIAM_CARTO_THREE_D);
    BOOST_TEST(c.map_rate_sec.count() == 1);
    BOOST_TEST(c.sensors.size() == 5);
    BOOST_TEST(c.sensors[0] == "sensor_1");
    BOOST_TEST(c.sensors[1] == "sensor_2");
    BOOST_TEST(c.sensors[2] == "sensor_3");
    BOOST_TEST(c.sensors[3] == "sensor_4");
    BOOST_TEST(c.sensors[4] == "sensor_5");

    viam_carto_config_teardown(vcc);

    fs::remove_all(tmp_dir);
    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_start_stop) {
    //  validate invalid pointer
    BOOST_TEST(viam_carto_start(nullptr) == VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_stop(nullptr) == VIAM_CARTO_VC_INVALID);

    // library init
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    // Setup
    viam_carto *vc;
    std::vector<std::string> sensors_vec;
    sensors_vec.push_back("sensor_1");
    sensors_vec.push_back("sensor_2");
    sensors_vec.push_back("sensor_3");
    sensors_vec.push_back("sensor_4");
    sensors_vec.push_back("sensor_5");
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),

                                "some component reference", sensors_vec);
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
    BOOST_TEST(cf->started == false);

    BOOST_TEST(fs::is_directory(cf->path_to_internal_state));
    BOOST_TEST(fs::is_empty(cf->path_to_internal_state));

    // // Start
    BOOST_TEST(viam_carto_start(vc) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(cf->started == true);

    // Confirm at least one map is persisted within the map_rate_sec
    VLOG(1) << "path_to_internal_state: " << cf->path_to_internal_state;
    // TODO: This should busy wait until this condition happens & the tests
    // passes or or 2 seconds goes by & the test fails.
    std::this_thread::sleep_for(cf->config.map_rate_sec +
                                std::chrono::seconds(1));
    BOOST_TEST(fs::is_directory(cf->path_to_internal_state));
    BOOST_TEST(!fs::is_empty(cf->path_to_internal_state));

    // Stop
    BOOST_TEST(viam_carto_stop(vc) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(cf->started == false);

    // Terminate
    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_SUCCESS);
    viam_carto_config_teardown(vcc);
    fs::remove_all(tmp_dir);

    // library terminate
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace carto_facade
}  // namespace viam
