#include "carto_facade.h"

#include <pcl/conversions.h>  // for pcl::fromPCLPointCloud2
#include <pcl/point_cloud.h>  // for pcl::PointCloud
#include <pcl/point_types.h>  // for pcl::PointXYZRGB

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <cstring>
#include <exception>
#include <filesystem>
#include <shared_mutex>
#include <string>

#include "bstrlib.h"
#include "glog/logging.h"
#include "test_helpers.h"
#include "util.h"

namespace tt = boost::test_tools;
namespace fs = std::filesystem;
namespace bfs = boost::filesystem;
namespace help = viam::carto_facade::test_helpers;
const auto tol = tt::tolerance(0.001);

namespace viam {
namespace carto_facade {
viam_carto_config viam_carto_config_setup(
    int map_rate_sec, viam_carto_LIDAR_CONFIG lidar_config,
    std::string data_dir, std::string camera, std::string movement_sensor,
    bool cloud_story_enabled, bool enable_mapping, std::string existing_map) {
    struct viam_carto_config vcc;
    vcc.map_rate_sec = map_rate_sec;
    vcc.lidar_config = lidar_config;
    vcc.data_dir = bfromcstr(data_dir.c_str());
    vcc.camera = bfromcstr(camera.c_str());
    vcc.movement_sensor = bfromcstr(movement_sensor.c_str());
    vcc.cloud_story_enabled = cloud_story_enabled;
    vcc.enable_mapping = enable_mapping;
    vcc.existing_map = bfromcstr(existing_map.c_str());
    return vcc;
}

void viam_carto_config_teardown(viam_carto_config vcc) {
    BOOST_TEST(bdestroy(vcc.data_dir) == BSTR_OK);
    BOOST_TEST(bdestroy(vcc.camera) == BSTR_OK);
    BOOST_TEST(bdestroy(vcc.movement_sensor) == BSTR_OK);
}
viam_carto_sensor_reading new_test_sensor_reading(
    std::string sensor, std::string pcd_path,
    int64_t sensor_reading_time_unix_milli) {
    viam_carto_sensor_reading sr;
    sr.sensor = bfromcstr(sensor.c_str());
    std::string pcd = help::read_file(pcd_path);
    sr.sensor_reading = blk2bstr(pcd.c_str(), pcd.length());
    BOOST_TEST(sr.sensor_reading != nullptr);
    sr.sensor_reading_time_unix_milli = sensor_reading_time_unix_milli;
    return sr;
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

BOOST_AUTO_TEST_SUITE(CartoFacadeCPPAPI);

BOOST_AUTO_TEST_CASE(CartoFacade_lib_init_terminate) {
    viam_carto_lib *lib;
    BOOST_TEST(FLAGS_logtostderr == 0);
    BOOST_TEST(viam_carto_lib_init(nullptr, 0, 0) == VIAM_CARTO_LIB_INVALID);
    BOOST_TEST(viam_carto_lib_terminate(nullptr) == VIAM_CARTO_LIB_INVALID);
    viam_carto_lib *invalidlib = nullptr;
    BOOST_TEST(viam_carto_lib_terminate(&invalidlib) == VIAM_CARTO_LIB_INVALID);

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
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());

    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    struct viam_carto_config vcc_empty_data_dir = viam_carto_config_setup(
        1, VIAM_CARTO_THREE_D, "", camera, movement_sensor, false, false, "");

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_empty_data_dir, ac) ==
               VIAM_CARTO_DATA_DIR_NOT_PROVIDED);

    std::string camera2 = "";
    std::string movement_sensor2 = "";

    struct viam_carto_config vcc_empty_component_ref =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),
                                camera2, movement_sensor2, false, false, "");

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_empty_component_ref, ac) ==
               VIAM_CARTO_COMPONENT_REFERENCE_INVALID);

    struct viam_carto_config vcc_invalid_map_rate_sec =
        viam_carto_config_setup(-1, VIAM_CARTO_THREE_D, tmp_dir.string(),
                                camera, movement_sensor, false, false, "");

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_invalid_map_rate_sec, ac) ==
               VIAM_CARTO_MAP_RATE_SEC_INVALID);

    struct viam_carto_config vcc_invalid_lidar_config = viam_carto_config_setup(
        1, static_cast<viam_carto_LIDAR_CONFIG>(-1), tmp_dir.string(), camera,
        movement_sensor, false, false, "");

    BOOST_TEST(viam_carto_init(&vc, lib, vcc_invalid_lidar_config, ac) ==
               VIAM_CARTO_LIDAR_CONFIG_INVALID);

    fs::path deprecated_path = tmp_dir / fs::path(bfs::unique_path().string());
    fs::create_directories(deprecated_path.string() + "/data");

    struct viam_carto_config vcc_deprecated_path =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, deprecated_path.string(),
                                camera, movement_sensor, false, false, "");
    BOOST_TEST(viam_carto_init(&vc, lib, vcc_deprecated_path, ac) ==
               VIAM_CARTO_DATA_DIR_INVALID_DEPRECATED_STRUCTURE);

    fs::path invalid_path = tmp_dir / fs::path(bfs::unique_path().string()) /
                            fs::path(bfs::unique_path().string());

    struct viam_carto_config vcc_invalid_path =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, invalid_path.string(),
                                camera, movement_sensor, false, false, "");
    BOOST_TEST(viam_carto_init(&vc, lib, vcc_invalid_path, ac) ==
               VIAM_CARTO_DATA_DIR_FILE_SYSTEM_ERROR);

    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(), camera,
                                movement_sensor, false, false, "");

    BOOST_TEST(viam_carto_init(nullptr, lib, vcc, ac) == VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_init(nullptr, nullptr, vcc, ac) ==
               VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_init(&vc, nullptr, vcc, ac) ==
               VIAM_CARTO_LIB_INVALID);

    // invalid invalid terminate
    BOOST_TEST(viam_carto_terminate(nullptr) == VIAM_CARTO_VC_INVALID);
    viam_carto *invalidvc = nullptr;
    BOOST_TEST(viam_carto_terminate(&invalidvc) == VIAM_CARTO_VC_INVALID);

    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(vc->slam_mode == VIAM_CARTO_SLAM_MODE_MAPPING);

    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_SUCCESS);
    // can't terminate a carto instance that has already been terminated
    BOOST_TEST(viam_carto_terminate(&vc) == VIAM_CARTO_VC_INVALID);

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
    // can't terminate a lib that has already been terminated
    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_LIB_INVALID);
}

BOOST_AUTO_TEST_CASE(CartoFacade_init_derive_slam_mode_cloud_story_enabled) {
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    {
        // mapping
        viam_carto *vc1;
        struct viam_carto_config vcc_mapping =
            viam_carto_config_setup(1, VIAM_CARTO_THREE_D, "",
                                    camera, movement_sensor, true, true, "");
        BOOST_TEST(viam_carto_init(&vc1, lib, vcc_mapping, ac) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc1->slam_mode == VIAM_CARTO_SLAM_MODE_MAPPING);
        viam::carto_facade::CartoFacade *cf1 =
            static_cast<viam::carto_facade::CartoFacade *>(vc1->carto_obj);
        BOOST_TEST(cf1->slam_mode == SlamMode::MAPPING);
        BOOST_TEST(cf1->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf1->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf1->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tol);
        BOOST_TEST(cf1->map_builder.GetMaxRange() == ac.max_range, tol);
        BOOST_TEST(cf1->map_builder.GetMinRange() == ac.min_range, tol);
        BOOST_TEST(cf1->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tol);
        BOOST_TEST(
            cf1->map_builder.GetTranslationWeight() == ac.translation_weight,
            tol);
        BOOST_TEST(cf1->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tol);
        // END TEST
        BOOST_TEST(viam_carto_terminate(&vc1) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_mapping);
    }

    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    fs::create_directory(tmp_dir);
    auto updating_dir = tmp_dir / fs::path("updating_dir");
    auto internal_state_file_path =
            updating_dir /
            fs::path("map_data_2022-02-11T01:44:53.1903Z.pbstream");
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
        fs::copy_file(internal_state_artifact_source,
                      internal_state_file_path);
    }

    {
        // updating
        viam_carto *vc2;

        struct viam_carto_config vcc_updating = viam_carto_config_setup(
            1, VIAM_CARTO_THREE_D, "", camera,
            movement_sensor, true, true, internal_state_file_path);
        BOOST_TEST(viam_carto_init(&vc2, lib, vcc_updating, ac) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc2->slam_mode == VIAM_CARTO_SLAM_MODE_UPDATING);
        viam::carto_facade::CartoFacade *cf2 =
            static_cast<viam::carto_facade::CartoFacade *>(vc2->carto_obj);
        BOOST_TEST(cf2->slam_mode == SlamMode::UPDATING);
        BOOST_TEST(cf2->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf2->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf2->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tol);
        BOOST_TEST(cf2->map_builder.GetMaxRange() == ac.max_range, tol);
        BOOST_TEST(cf2->map_builder.GetMinRange() == ac.min_range, tol);
        BOOST_TEST(cf2->map_builder.GetFreshSubmapsCount() ==
                   ac.fresh_submaps_count);
        BOOST_TEST(cf2->map_builder.GetMinCoveredArea() == ac.min_covered_area,
                   tol);
        BOOST_TEST(cf2->map_builder.GetMinAddedSubmapsCount() ==
                   ac.min_added_submaps_count);
        BOOST_TEST(cf2->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tol);
        BOOST_TEST(
            cf2->map_builder.GetTranslationWeight() == ac.translation_weight,
            tol);
        BOOST_TEST(cf2->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tol);
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
            1, VIAM_CARTO_THREE_D, "", camera,
            movement_sensor, true, true, internal_state_file_path);

        BOOST_TEST(viam_carto_init(&vc3, lib, vcc_updating,
                                   ac_optimize_on_start) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc3->slam_mode == VIAM_CARTO_SLAM_MODE_UPDATING);
        viam::carto_facade::CartoFacade *cf2 =
            static_cast<viam::carto_facade::CartoFacade *>(vc3->carto_obj);
        BOOST_TEST(cf2->slam_mode == SlamMode::UPDATING);
        BOOST_TEST(viam_carto_terminate(&vc3) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_updating);
    }

    {
        // localizing
        viam_carto *vc4;
        struct viam_carto_config vcc_localizing = viam_carto_config_setup(
            0, VIAM_CARTO_THREE_D, "", camera,
            movement_sensor, true, false, internal_state_file_path);
        BOOST_TEST(viam_carto_init(&vc4, lib, vcc_localizing, ac) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc4->slam_mode == VIAM_CARTO_SLAM_MODE_LOCALIZING);
        viam::carto_facade::CartoFacade *cf3 =
            static_cast<viam::carto_facade::CartoFacade *>(vc4->carto_obj);
        BOOST_TEST(cf3->slam_mode == SlamMode::LOCALIZING);
        BOOST_TEST(cf3->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf3->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf3->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tol);
        BOOST_TEST(cf3->map_builder.GetMaxRange() == ac.max_range, tol);
        BOOST_TEST(cf3->map_builder.GetMinRange() == ac.min_range, tol);
        BOOST_TEST(cf3->map_builder.GetMaxSubmapsToKeep() ==
                   ac.max_submaps_to_keep);
        BOOST_TEST(cf3->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tol);
        BOOST_TEST(
            cf3->map_builder.GetTranslationWeight() == ac.translation_weight,
            tol);
        BOOST_TEST(cf3->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tol);
        BOOST_TEST(viam_carto_terminate(&vc4) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_localizing);
    }

    {
        // localizing optimize_on_start
        viam_carto *vc5;
        struct viam_carto_config vcc_localizing = viam_carto_config_setup(
            0, VIAM_CARTO_THREE_D, "", camera,
            movement_sensor, true, false, internal_state_file_path);
        BOOST_TEST(viam_carto_init(&vc5, lib, vcc_localizing,
                                   ac_optimize_on_start) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc5->slam_mode == VIAM_CARTO_SLAM_MODE_LOCALIZING);
        viam::carto_facade::CartoFacade *cf3 =
            static_cast<viam::carto_facade::CartoFacade *>(vc5->carto_obj);
        BOOST_TEST(cf3->slam_mode == SlamMode::LOCALIZING);
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
                                    camera, movement_sensor, false, false, "test.pbstream");
        BOOST_TEST(viam_carto_init(&vc6, lib, vcc_invalid, ac) ==
                   VIAM_CARTO_SLAM_MODE_INVALID);
        viam_carto_config_teardown(vcc_invalid);
    }

    // TODO: Move all suite level setup & teardown to boost test hook
    fs::remove_all(tmp_dir);

    BOOST_TEST(viam_carto_lib_terminate(&lib) == VIAM_CARTO_SUCCESS);
}

BOOST_AUTO_TEST_CASE(CartoFacade_init_derive_slam_mode) {
    viam_carto_lib *lib;
    BOOST_TEST(viam_carto_lib_init(&lib, 0, 1) == VIAM_CARTO_SUCCESS);

    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    fs::create_directory(tmp_dir);
    {
        // mapping
        viam_carto *vc1;
        auto mapping_dir = tmp_dir / fs::path("mapping_dir");
        struct viam_carto_config vcc_mapping =
            viam_carto_config_setup(1, VIAM_CARTO_THREE_D, mapping_dir.string(),
                                    camera, movement_sensor, false, false, "");
        BOOST_TEST(viam_carto_init(&vc1, lib, vcc_mapping, ac) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc1->slam_mode == VIAM_CARTO_SLAM_MODE_MAPPING);
        viam::carto_facade::CartoFacade *cf1 =
            static_cast<viam::carto_facade::CartoFacade *>(vc1->carto_obj);
        BOOST_TEST(cf1->slam_mode == SlamMode::MAPPING);
        BOOST_TEST(cf1->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf1->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf1->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tol);
        BOOST_TEST(cf1->map_builder.GetMaxRange() == ac.max_range, tol);
        BOOST_TEST(cf1->map_builder.GetMinRange() == ac.min_range, tol);
        BOOST_TEST(cf1->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tol);
        BOOST_TEST(
            cf1->map_builder.GetTranslationWeight() == ac.translation_weight,
            tol);
        BOOST_TEST(cf1->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tol);
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
            1, VIAM_CARTO_THREE_D, updating_dir.string(), camera,
            movement_sensor, false, false, "");
        BOOST_TEST(viam_carto_init(&vc2, lib, vcc_updating, ac) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc2->slam_mode == VIAM_CARTO_SLAM_MODE_UPDATING);
        viam::carto_facade::CartoFacade *cf2 =
            static_cast<viam::carto_facade::CartoFacade *>(vc2->carto_obj);
        BOOST_TEST(cf2->slam_mode == SlamMode::UPDATING);
        BOOST_TEST(cf2->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf2->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf2->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tol);
        BOOST_TEST(cf2->map_builder.GetMaxRange() == ac.max_range, tol);
        BOOST_TEST(cf2->map_builder.GetMinRange() == ac.min_range, tol);
        BOOST_TEST(cf2->map_builder.GetFreshSubmapsCount() ==
                   ac.fresh_submaps_count);
        BOOST_TEST(cf2->map_builder.GetMinCoveredArea() == ac.min_covered_area,
                   tol);
        BOOST_TEST(cf2->map_builder.GetMinAddedSubmapsCount() ==
                   ac.min_added_submaps_count);
        BOOST_TEST(cf2->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tol);
        BOOST_TEST(
            cf2->map_builder.GetTranslationWeight() == ac.translation_weight,
            tol);
        BOOST_TEST(cf2->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tol);
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
            1, VIAM_CARTO_THREE_D, updating_dir.string(), camera,
            movement_sensor, false, false, "");

        BOOST_TEST(viam_carto_init(&vc3, lib, vcc_updating,
                                   ac_optimize_on_start) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc3->slam_mode == VIAM_CARTO_SLAM_MODE_UPDATING);
        viam::carto_facade::CartoFacade *cf2 =
            static_cast<viam::carto_facade::CartoFacade *>(vc3->carto_obj);
        BOOST_TEST(cf2->slam_mode == SlamMode::UPDATING);
        BOOST_TEST(viam_carto_terminate(&vc3) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_updating);
    }

    {
        // localizing
        viam_carto *vc4;
        struct viam_carto_config vcc_localizing = viam_carto_config_setup(
            0, VIAM_CARTO_THREE_D, updating_dir.string(), camera,
            movement_sensor, false, false, "");
        BOOST_TEST(viam_carto_init(&vc4, lib, vcc_localizing, ac) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc4->slam_mode == VIAM_CARTO_SLAM_MODE_LOCALIZING);
        viam::carto_facade::CartoFacade *cf3 =
            static_cast<viam::carto_facade::CartoFacade *>(vc4->carto_obj);
        BOOST_TEST(cf3->slam_mode == SlamMode::LOCALIZING);
        BOOST_TEST(cf3->map_builder.GetOptimizeEveryNNodes() ==
                   ac.optimize_every_n_nodes);
        BOOST_TEST(cf3->map_builder.GetNumRangeData() == ac.num_range_data);
        BOOST_TEST(cf3->map_builder.GetMissingDataRayLength() ==
                       ac.missing_data_ray_length,
                   tol);
        BOOST_TEST(cf3->map_builder.GetMaxRange() == ac.max_range, tol);
        BOOST_TEST(cf3->map_builder.GetMinRange() == ac.min_range, tol);
        BOOST_TEST(cf3->map_builder.GetMaxSubmapsToKeep() ==
                   ac.max_submaps_to_keep);
        BOOST_TEST(cf3->map_builder.GetOccupiedSpaceWeight() ==
                       ac.occupied_space_weight,
                   tol);
        BOOST_TEST(
            cf3->map_builder.GetTranslationWeight() == ac.translation_weight,
            tol);
        BOOST_TEST(cf3->map_builder.GetRotationWeight() == ac.rotation_weight,
                   tol);
        BOOST_TEST(viam_carto_terminate(&vc4) == VIAM_CARTO_SUCCESS);
        viam_carto_config_teardown(vcc_localizing);
    }

    {
        // localizing optimize_on_start
        viam_carto *vc5;
        struct viam_carto_config vcc_localizing = viam_carto_config_setup(
            0, VIAM_CARTO_THREE_D, updating_dir.string(), camera,
            movement_sensor, false, false, "");
        BOOST_TEST(viam_carto_init(&vc5, lib, vcc_localizing,
                                   ac_optimize_on_start) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(vc5->slam_mode == VIAM_CARTO_SLAM_MODE_LOCALIZING);
        viam::carto_facade::CartoFacade *cf3 =
            static_cast<viam::carto_facade::CartoFacade *>(vc5->carto_obj);
        BOOST_TEST(cf3->slam_mode == SlamMode::LOCALIZING);
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
                                    camera, movement_sensor, false, false, "");
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
    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(), camera,
                                movement_sensor, false, false, "");
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();
    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(vc->slam_mode == VIAM_CARTO_SLAM_MODE_MAPPING);
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
    BOOST_TEST((cf->lib) == lib);
    BOOST_TEST((cf->algo_config.optimize_on_start) == false);
    BOOST_TEST((cf->algo_config.optimize_every_n_nodes) == 3);
    BOOST_TEST((cf->algo_config.num_range_data) == 100);
    BOOST_TEST((cf->algo_config.missing_data_ray_length) == 25, tol);
    BOOST_TEST((cf->algo_config.max_range) == 25, tol);
    BOOST_TEST((cf->algo_config.min_range) == 0.2, tol);
    BOOST_TEST((cf->algo_config.max_submaps_to_keep) == 3);
    BOOST_TEST((cf->algo_config.fresh_submaps_count) == 3);
    BOOST_TEST((cf->algo_config.min_covered_area) == 1, tol);
    BOOST_TEST((cf->algo_config.min_added_submaps_count) == 1);
    BOOST_TEST((cf->algo_config.occupied_space_weight) == 20, tol);
    BOOST_TEST((cf->algo_config.translation_weight) == 10, tol);
    BOOST_TEST((cf->algo_config.rotation_weight) == 1, tol);
    auto path_to_internal_state = tmp_dir / fs::path("internal_state");

    BOOST_TEST((cf->path_to_internal_state) == path_to_internal_state.string());
    BOOST_TEST(((cf->state) == CartoFacadeState::IO_INITIALIZED));
    BOOST_TEST((cf->config.camera) == camera);
    BOOST_TEST((cf->config.movement_sensor) == movement_sensor);
    BOOST_TEST((cf->config.map_rate_sec).count() == 1);
    BOOST_TEST((cf->config.data_dir) == tmp_dir.string());
    BOOST_TEST(to_std_string(cf->config.component_reference) == "lidar");
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
    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(60, VIAM_CARTO_THREE_D, tmp_dir.string(),

                                camera, movement_sensor, false, false, "");
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(vc->slam_mode == VIAM_CARTO_SLAM_MODE_MAPPING);

    // behavior of methods before start
    // AddSensorReading
    {
        viam_carto_sensor_reading sr = new_test_sensor_reading(
            "lidar", ".artifact/data/viam-cartographer/mock_lidar/0.pcd",
            1687900053773475);
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    {
        // GetPosition
        viam_carto_get_position_response pr;
        BOOST_TEST(viam_carto_get_position(vc, &pr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
    }

    //  GetPointCloudMap
    {
        viam_carto_get_point_cloud_map_response mr;
        BOOST_TEST(viam_carto_get_point_cloud_map(vc, &mr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
    }

    // GetInternalState
    {
        viam_carto_get_internal_state_response isr;
        BOOST_TEST(viam_carto_get_internal_state(vc, &isr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
    }

    // Start
    BOOST_TEST(viam_carto_start(vc) == VIAM_CARTO_SUCCESS);
    // start not allowed if already started
    BOOST_TEST(viam_carto_start(vc) == VIAM_CARTO_NOT_IN_IO_INITIALIZED_STATE);

    // can't terminate a carto instance which is still started
    BOOST_TEST(viam_carto_terminate(&vc) ==
               VIAM_CARTO_NOT_IN_TERMINATABLE_STATE);

    // GetPosition
    BOOST_TEST(viam_carto_get_position(nullptr, nullptr) ==
               VIAM_CARTO_VC_INVALID);
    BOOST_TEST(viam_carto_get_position(vc, nullptr) ==
               VIAM_CARTO_GET_POSITION_RESPONSE_INVALID);
    BOOST_TEST(viam_carto_get_position_response_destroy(nullptr) ==
               VIAM_CARTO_GET_POSITION_RESPONSE_INVALID);
    {
        viam_carto_get_position_response pr;
        // Test get position before any data is provided
        // it should be all zeroed out
        BOOST_TEST(viam_carto_get_position(vc, &pr) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(pr.x == 0);
        BOOST_TEST(pr.y == 0);
        BOOST_TEST(pr.z == 0);
        BOOST_TEST(pr.imag == 0);
        BOOST_TEST(pr.jmag == 0);
        BOOST_TEST(pr.kmag == 0);
        BOOST_TEST(pr.real == 1);
        BOOST_TEST(to_std_string(pr.component_reference) == "lidar");

        BOOST_TEST(viam_carto_get_position_response_destroy(&pr) ==
                   VIAM_CARTO_SUCCESS);
    }

    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};
    // vc nullptr
    {
        BOOST_TEST(viam_carto_add_sensor_reading(nullptr, nullptr) ==
                   VIAM_CARTO_VC_INVALID);
    }

    // viam_carto_sensor_reading nullptr
    {
        BOOST_TEST(viam_carto_add_sensor_reading(vc, nullptr) ==
                   VIAM_CARTO_SENSOR_READING_INVALID);
    }
    BOOST_TEST(viam_carto_add_sensor_reading_destroy(nullptr) ==
               VIAM_CARTO_SENSOR_READING_INVALID);

    {
        viam_carto_sensor_reading sr;
        // must be they first sensor in the sensor list
        sr.sensor = bfromcstr("never heard of it sensor");
        std::string pcd = help::binary_pcd(points);
        sr.sensor_reading = blk2bstr(pcd.c_str(), pcd.length());
        BOOST_TEST(sr.sensor_reading != nullptr);
        sr.sensor_reading_time_unix_milli = 1687899990420347;
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // non first sensor
    {
        viam_carto_sensor_reading sr;
        // must be they first sensor in the sensor list
        sr.sensor = bfromcstr("sensor_2");
        std::string pcd = help::binary_pcd(points);
        sr.sensor_reading = blk2bstr(pcd.c_str(), pcd.length());
        BOOST_TEST(sr.sensor_reading != nullptr);
        sr.sensor_reading_time_unix_milli = 1687900014152474;
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // empty sensor reading
    {
        viam_carto_sensor_reading sr;
        // must be they first sensor in the sensor list
        sr.sensor = bfromcstr("lidar");
        std::string pcd = "empty lidar reading";
        // passing 0 as the second parameter makes the string empty
        sr.sensor_reading = blk2bstr(pcd.c_str(), 0);
        BOOST_TEST(sr.sensor_reading != nullptr);
        sr.sensor_reading_time_unix_milli = 1687900021820215;
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SENSOR_READING_EMPTY);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // invalid reading
    {
        viam_carto_sensor_reading sr;
        // must be they first sensor in the sensor list
        sr.sensor = bfromcstr("lidar");
        std::string pcd = "invalid lidar reading";
        sr.sensor_reading = blk2bstr(pcd.c_str(), pcd.length());
        BOOST_TEST(sr.sensor_reading != nullptr);
        sr.sensor_reading_time_unix_milli = 1687900029557335;
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SENSOR_READING_INVALID);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // unable to aquire lock
    {
        viam_carto_sensor_reading sr = new_test_sensor_reading(
            "lidar", ".artifact/data/viam-cartographer/mock_lidar/0.pcd",
            1687900053773475);
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        std::lock_guard<std::mutex> lk(cf->map_builder_mutex);
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // GetPointCloudMap before successful sensor readings
    {
        BOOST_TEST(viam_carto_get_point_cloud_map_response_destroy(nullptr) ==
                   VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVLALID);
        BOOST_TEST(viam_carto_get_point_cloud_map(nullptr, nullptr) ==
                   VIAM_CARTO_VC_INVALID);
        BOOST_TEST(viam_carto_get_point_cloud_map(vc, nullptr) ==
                   VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVLALID);

        viam_carto_get_point_cloud_map_response mr;
        BOOST_TEST(viam_carto_get_point_cloud_map(vc, &mr) ==
                   VIAM_CARTO_POINTCLOUD_MAP_EMPTY);
    }

    // GetInternalState
    int last_internal_state_response_size = 0;
    {
        BOOST_TEST(viam_carto_get_internal_state_response_destroy(nullptr) ==
                   VIAM_CARTO_GET_INTERNAL_STATE_RESPONSE_INVLALID);
        BOOST_TEST(viam_carto_get_internal_state(nullptr, nullptr) ==
                   VIAM_CARTO_VC_INVALID);
        BOOST_TEST(viam_carto_get_internal_state(vc, nullptr) ==
                   VIAM_CARTO_GET_INTERNAL_STATE_RESPONSE_INVLALID);

        viam_carto_get_internal_state_response isr;
        BOOST_TEST(viam_carto_get_internal_state(vc, &isr) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(blength(isr.internal_state) >
                   last_internal_state_response_size);
        last_internal_state_response_size = blength(isr.internal_state);
        BOOST_TEST(viam_carto_get_internal_state_response_destroy(&isr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // GetPosition unchanged from failed AddSensorReading requests
    {
        viam_carto_get_position_response pr;
        // Test get position before any data is provided
        // it should be all zeroed out
        BOOST_TEST(viam_carto_get_position(vc, &pr) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(pr.x == 0);
        BOOST_TEST(pr.y == 0);
        BOOST_TEST(pr.z == 0);
        BOOST_TEST(pr.imag == 0);
        BOOST_TEST(pr.jmag == 0);
        BOOST_TEST(pr.kmag == 0);
        BOOST_TEST(pr.real == 1);
        BOOST_TEST(to_std_string(pr.component_reference) == "lidar");

        BOOST_TEST(viam_carto_get_position_response_destroy(&pr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // GetPosition is unchanged from first AddSensorReading request
    // as cartographer needs at least 3 lidar readings to compute a
    // position and the first position computed is zero.
    // As a result it takes a minimum of 3 lidar readings before
    // cartographer produces a non zeroed position.
    // For more info see:
    // https://github.com/cartographer-project/cartographer/blob/ef00de2317dcf7895b09f18cc4d87f8b533a019b/cartographer/mapping/internal/global_trajectory_builder.cc#L65
    // https://github.com/cartographer-project/cartographer/blob/ef00de2317dcf7895b09f18cc4d87f8b533a019b/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc#L136

    // first sensor reading
    {
        VLOG(1) << "viam_carto_add_sensor_reading 1";
        viam_carto_sensor_reading sr = new_test_sensor_reading(
            "lidar", ".artifact/data/viam-cartographer/mock_lidar/0.pcd",
            1629037851000000);
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    {
        viam_carto_get_position_response pr;
        BOOST_TEST(viam_carto_get_position(vc, &pr) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(pr.x == 0);
        BOOST_TEST(pr.y == 0);
        BOOST_TEST(pr.z == 0);
        BOOST_TEST(pr.imag == 0);
        BOOST_TEST(pr.jmag == 0);
        BOOST_TEST(pr.kmag == 0);
        BOOST_TEST(pr.real == 1);
        BOOST_TEST(to_std_string(pr.component_reference) == "lidar");

        BOOST_TEST(viam_carto_get_position_response_destroy(&pr) ==
                   VIAM_CARTO_SUCCESS);
    }

    {
        viam_carto_get_point_cloud_map_response mr;
        BOOST_TEST(viam_carto_get_point_cloud_map(vc, &mr) ==
                   VIAM_CARTO_POINTCLOUD_MAP_EMPTY);
    }

    {
        viam_carto_get_internal_state_response isr;
        BOOST_TEST(viam_carto_get_internal_state(vc, &isr) ==
                   VIAM_CARTO_SUCCESS);
        // special case: the first call to AddSensorReading doesn't
        // change the internal state but subsequent calls do.
        BOOST_TEST(blength(isr.internal_state) ==
                   last_internal_state_response_size);
        last_internal_state_response_size = blength(isr.internal_state);
        BOOST_TEST(viam_carto_get_internal_state_response_destroy(&isr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // second sensor reading
    {
        VLOG(1) << "viam_carto_add_sensor_reading 2";
        viam_carto_sensor_reading sr = new_test_sensor_reading(
            "lidar", ".artifact/data/viam-cartographer/mock_lidar/1.pcd",
            1629037853000000);
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // GetPosition returning origin position from 2 successful AddSensorReading
    // requests
    {
        viam_carto_get_position_response pr;
        BOOST_TEST(viam_carto_get_position(vc, &pr) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(pr.x == 0);
        BOOST_TEST(pr.y == 0);
        BOOST_TEST(pr.z == 0);
        BOOST_TEST(pr.imag == 0);
        BOOST_TEST(pr.jmag == 0);
        BOOST_TEST(pr.kmag == 0);
        BOOST_TEST(pr.real == 1);
        BOOST_TEST(to_std_string(pr.component_reference) == "lidar");

        BOOST_TEST(viam_carto_get_position_response_destroy(&pr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // GetPointCloudMap after successful sensor readings
    {
        viam_carto_get_point_cloud_map_response mr;
        BOOST_TEST(viam_carto_get_point_cloud_map(vc, &mr) ==
                   VIAM_CARTO_SUCCESS);
        pcl::PCLPointCloud2 blob;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        auto s = to_std_string(mr.point_cloud_pcd);
        BOOST_TEST(viam::carto_facade::util::read_pcd(s, blob) == 0);
        pcl::fromPCLPointCloud2(blob, *cloud);
        BOOST_TEST(cloud != nullptr);
        BOOST_TEST(cloud->points.size() != 0);
        BOOST_TEST(viam_carto_get_point_cloud_map_response_destroy(&mr) ==
                   VIAM_CARTO_SUCCESS);
    }

    {
        viam_carto_get_internal_state_response isr;
        BOOST_TEST(viam_carto_get_internal_state(vc, &isr) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(blength(isr.internal_state) >
                   last_internal_state_response_size);
        last_internal_state_response_size = blength(isr.internal_state);
        BOOST_TEST(viam_carto_get_internal_state_response_destroy(&isr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // third sensor reading
    {
        VLOG(1) << "viam_carto_add_sensor_reading 3";
        viam_carto_sensor_reading sr = new_test_sensor_reading(
            "lidar", ".artifact/data/viam-cartographer/mock_lidar/2.pcd",
            1629037855000000);
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_SUCCESS);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // GetPosition changed from 3 successful AddSensorReading requests
    {
        viam_carto_get_position_response pr;
        BOOST_TEST(viam_carto_get_position(vc, &pr) == VIAM_CARTO_SUCCESS);
        BOOST_TEST(pr.x != 0);
        BOOST_TEST(pr.y != 0);
        BOOST_TEST(pr.z == 0);
        BOOST_TEST(pr.imag == 0);
        BOOST_TEST(pr.jmag == 0);
        BOOST_TEST(pr.kmag != 0);
        BOOST_TEST(pr.real != 1);
        BOOST_TEST(to_std_string(pr.component_reference) == "lidar");

        BOOST_TEST(viam_carto_get_position_response_destroy(&pr) ==
                   VIAM_CARTO_SUCCESS);
    }

    {
        viam_carto_get_internal_state_response isr;
        BOOST_TEST(viam_carto_get_internal_state(vc, &isr) ==
                   VIAM_CARTO_SUCCESS);
        // on arm64 linux this is strictly greater than
        // on arm64 mac for some reason it is equal to
        // https://viam.atlassian.net/browse/RSDK-3866
        BOOST_TEST(blength(isr.internal_state) >=
                   last_internal_state_response_size);
        last_internal_state_response_size = blength(isr.internal_state);
        BOOST_TEST(viam_carto_get_internal_state_response_destroy(&isr) ==
                   VIAM_CARTO_SUCCESS);
    }

    // Stop
    BOOST_TEST(viam_carto_stop(vc) == VIAM_CARTO_SUCCESS);
    // stop not allowed if not started
    BOOST_TEST(viam_carto_stop(vc) == VIAM_CARTO_NOT_IN_STARTED_STATE);

    // behavior of methods after stop
    // AddSensorReading
    {
        viam_carto_sensor_reading sr = new_test_sensor_reading(
            "lidar", ".artifact/data/viam-cartographer/mock_lidar/0.pcd",
            1687900053773475);
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        BOOST_TEST(viam_carto_add_sensor_reading(vc, &sr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
        BOOST_TEST(viam_carto_add_sensor_reading_destroy(&sr) ==
                   VIAM_CARTO_SUCCESS);
    }

    {
        // GetPosition
        viam_carto_get_position_response pr;
        BOOST_TEST(viam_carto_get_position(vc, &pr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
    }

    //  GetPointCloudMap
    {
        viam_carto_get_point_cloud_map_response mr;
        BOOST_TEST(viam_carto_get_point_cloud_map(vc, &mr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
    }

    // GetInternalState
    {
        viam_carto_get_internal_state_response isr;
        BOOST_TEST(viam_carto_get_internal_state(vc, &isr) ==
                   VIAM_CARTO_NOT_IN_STARTED_STATE);
    }

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

    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),

                                camera, movement_sensor, true, false, "");

    struct config c = viam::carto_facade::from_viam_carto_config(vcc);

    BOOST_TEST(to_std_string(c.component_reference) == "lidar");
    BOOST_TEST(c.data_dir == tmp_dir.string());
    BOOST_TEST(c.lidar_config == VIAM_CARTO_THREE_D);
    BOOST_TEST(c.map_rate_sec.count() == 1);
    BOOST_TEST(c.camera == "lidar");
    BOOST_TEST(c.movement_sensor == "imu");
    BOOST_TEST(c.cloud_story_enabled == true);
    BOOST_TEST(c.enable_mapping == false);

    viam_carto_config_teardown(vcc);
    BOOST_TEST(bdestroy(c.component_reference) == BSTR_OK);

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
    std::string camera = "lidar";
    std::string movement_sensor = "imu";
    fs::path tmp_dir =
        fs::temp_directory_path() / fs::path(bfs::unique_path().string());
    struct viam_carto_config vcc =
        viam_carto_config_setup(1, VIAM_CARTO_THREE_D, tmp_dir.string(),

                                camera, movement_sensor, false, false, "");
    struct viam_carto_algo_config ac = viam_carto_algo_config_setup();

    BOOST_TEST(viam_carto_init(&vc, lib, vcc, ac) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(vc->slam_mode == VIAM_CARTO_SLAM_MODE_MAPPING);
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
    BOOST_TEST(((cf->state) == CartoFacadeState::IO_INITIALIZED));

    BOOST_TEST(fs::is_directory(cf->path_to_internal_state));
    BOOST_TEST(fs::is_empty(cf->path_to_internal_state));

    // // Start
    BOOST_TEST(viam_carto_start(vc) == VIAM_CARTO_SUCCESS);
    BOOST_TEST(((cf->state) == CartoFacadeState::STARTED));

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
    BOOST_TEST(((cf->state) == CartoFacadeState::IO_INITIALIZED));

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
