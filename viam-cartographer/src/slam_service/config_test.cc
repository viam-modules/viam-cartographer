#include "config.h"

#include <boost/test/unit_test.hpp>
#include <exception>

namespace viam {
namespace config {
namespace {

BOOST_AUTO_TEST_SUITE(Config)

void checkParseAndValidateConfigParamsException(int argc, char** argv,
                                                const std::string& message) {
    SLAMServiceImpl slamService;
    BOOST_CHECK_EXCEPTION(
        ParseAndValidateConfigParams(argc, argv, &slamService),
        std::runtime_error, [&message](const std::runtime_error& ex) {
            BOOST_CHECK_EQUAL(ex.what(), message);
            return true;
        });
}

char** toCharArrayArray(std::vector<std::string>& args) {
    char** argv = new char*[args.size()];
    for (auto i = 0; i < args.size(); i++) {
        argv[i] = &args[i][0];
    }
    return argv;
}

template <typename T>
void checkCartoConfigParams(SLAMServiceImpl& slamService, std::string parameter,
                            T parameter_value) {
    auto tolerance = boost::test_tools::tolerance(0.00001);
    if (parameter == "optimize_every_n_nodes") {
        BOOST_TEST(slamService.optimize_every_n_nodes == parameter_value);
    } else if (parameter == "num_range_data") {
        BOOST_TEST(slamService.num_range_data == parameter_value);
    } else if (parameter == "missing_data_ray_length") {
        BOOST_TEST(slamService.missing_data_ray_length == parameter_value,
                   tolerance);
    } else if (parameter == "max_range") {
        BOOST_TEST(slamService.max_range == parameter_value, tolerance);
    } else if (parameter == "min_range") {
        BOOST_TEST(slamService.min_range == parameter_value, tolerance);
    } else if (parameter == "max_submaps_to_keep") {
        BOOST_TEST(slamService.max_submaps_to_keep == parameter_value);
    } else if (parameter == "fresh_submaps_count") {
        BOOST_TEST(slamService.fresh_submaps_count == parameter_value);
    } else if (parameter == "min_covered_area") {
        BOOST_TEST(slamService.min_covered_area == parameter_value, tolerance);
    } else if (parameter == "min_added_submaps_count") {
        BOOST_TEST(slamService.min_added_submaps_count == parameter_value);
    } else if (parameter == "occupied_space_weight") {
        BOOST_TEST(slamService.occupied_space_weight == parameter_value,
                   tolerance);
    } else if (parameter == "translation_weight") {
        BOOST_TEST(slamService.translation_weight == parameter_value,
                   tolerance);
    } else if (parameter == "rotation_weight") {
        BOOST_TEST(slamService.rotation_weight == parameter_value, tolerance);
    }
}

BOOST_AUTO_TEST_CASE(OverwriteCartoConfigParam_invalid_parameter) {
    SLAMServiceImpl slamService;
    std::string parameter = "invalid_parameter";
    int parameter_value = 999;
    slamService.config_params =
        "{mode=2d," + parameter + "=" + std::to_string(parameter_value) + "}";
    const std::string message =
        "unsupported cartographer config parameter: " + parameter;
    BOOST_CHECK_EXCEPTION(OverwriteCartoConfigParam(&slamService, parameter),
                          std::runtime_error,
                          [&message](const std::runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });
}

BOOST_AUTO_TEST_CASE(OverwriteCartoConfigParam_valid_parameters) {
    SLAMServiceImpl slamService;
    std::map<std::string, int> param_value_map_ints{
        {"optimize_every_n_nodes", 1},
        {"num_range_data", 2},
        {"max_submaps_to_keep", 33},
        {"fresh_submaps_count", 4},
        {"min_added_submaps_count", 5}};
    std::map<std::string, float> param_value_map_floats{
        {"missing_data_ray_length", 6.0},
        {"max_range", 7.0},
        {"min_range", 8.0}};
    std::map<std::string, double> param_value_map_doubles{
        {"min_covered_area", 9.0},
        {"occupied_space_weight", 10.0},
        {"rotation_weight", 11.0}};
    for (auto const& x : param_value_map_ints) {
        std::string parameter = x.first;
        int parameter_value = x.second;
        slamService.config_params = "{mode=2d," + parameter + "=" +
                                    std::to_string(parameter_value) + "}";
        OverwriteCartoConfigParam(&slamService, parameter);
        checkCartoConfigParams<int>(slamService, parameter, parameter_value);
    }
    for (auto const& x : param_value_map_floats) {
        std::string parameter = x.first;
        int parameter_value = x.second;
        slamService.config_params = "{mode=2d," + parameter + "=" +
                                    std::to_string(parameter_value) + "}";
        OverwriteCartoConfigParam(&slamService, parameter);
        checkCartoConfigParams<float>(slamService, parameter, parameter_value);
    }
    for (auto const& x : param_value_map_doubles) {
        std::string parameter = x.first;
        int parameter_value = x.second;
        slamService.config_params = "{mode=2d," + parameter + "=" +
                                    std::to_string(parameter_value) + "}";
        OverwriteCartoConfigParam(&slamService, parameter);
        checkCartoConfigParams<double>(slamService, parameter, parameter_value);
    }
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_config_param) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server",
                                  "-data_dir=/path/to",
                                  "-port=localhost:0",
                                  "-sensors=lidar",
                                  "-data_rate_ms=200",
                                  "-map_rate_sec=60",
                                  "-delete_processed_data=false",
                                  "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "-config_param is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_data_dir) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server",
                                  "-config_param={mode=2d}",
                                  "-port=localhost:0",
                                  "-sensors=lidar",
                                  "-data_rate_ms=200",
                                  "-map_rate_sec=60",
                                  "-delete_processed_data=false",
                                  "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "-data_dir is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_port) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server",
                                  "-config_param={mode=2d}",
                                  "-data_dir=/path/to",
                                  "-sensors=lidar",
                                  "-data_rate_ms=200",
                                  "-map_rate_sec=60",
                                  "-delete_processed_data=false",
                                  "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "-port is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_no_slam_mode) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-data_rate_ms=200",
        "-map_rate_sec=60",   "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "slam mode is missing";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_invalid_slam_mode) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=bad}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-data_rate_ms=200",
        "-map_rate_sec=60",   "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    const std::string message = "Invalid slam_mode=bad";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_valid_config) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2d}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-data_rate_ms=200",
        "-map_rate_sec=60",   "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    config::ParseAndValidateConfigParams(argc, argv, &slamService);

    auto tolerance = boost::test_tools::tolerance(0.00001);
    BOOST_TEST(slamService.path_to_data == "/path/to/data");
    BOOST_TEST(slamService.path_to_map == "/path/to/map");
    BOOST_TEST(slamService.config_params == "{mode=2d}");
    BOOST_TEST(slamService.slam_mode == "2d");
    BOOST_TEST(slamService.optimize_every_n_nodes == 3);
    BOOST_TEST(slamService.num_range_data == 100);
    BOOST_TEST(slamService.missing_data_ray_length == 25.0, tolerance);
    BOOST_TEST(slamService.max_range == 25.0, tolerance);
    BOOST_TEST(slamService.min_range == 0.2, tolerance);
    BOOST_TEST(slamService.max_submaps_to_keep == 3);  // LOCALIZATION only
    BOOST_TEST(slamService.fresh_submaps_count == 3);  // UPDATING only
    BOOST_TEST(slamService.min_covered_area == 1.0,
               tolerance);                                 // UPDATING only
    BOOST_TEST(slamService.min_added_submaps_count == 1);  // UPDATING only
    BOOST_TEST(slamService.occupied_space_weight == 20.0, tolerance);
    BOOST_TEST(slamService.translation_weight == 10.0, tolerance);
    BOOST_TEST(slamService.rotation_weight == 1.0, tolerance);
    BOOST_TEST(slamService.port == "localhost:0");
    BOOST_TEST(slamService.data_rate_ms.count() ==
               std::chrono::milliseconds(200).count());
    BOOST_TEST(slamService.map_rate_sec.count() ==
               std::chrono::seconds(60).count());
    BOOST_TEST(slamService.camera_name == "lidar");
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.optimize_on_start == false);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_with_carto_params) {
    ResetFlagsForTesting();
    std::string config_param =
        "{mode=2d,"
        "optimize_on_start=true,"
        "optimize_every_n_nodes=9000,num_range_data=9001,"
        "missing_data_ray_length=9002.2,max_range=9003.3,"
        "min_range=9004.4,max_submaps_to_keep=9005,"
        "fresh_submaps_count=9006,min_covered_area=9007.7,"
        "min_added_submaps_count=9008,occupied_space_weight=9009.9,"
        "translation_weight=9010.1,rotation_weight=9011.1}";
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param=" + config_param,
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-data_rate_ms=200",
        "-map_rate_sec=60",   "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    config::ParseAndValidateConfigParams(argc, argv, &slamService);

    auto tolerance = boost::test_tools::tolerance(0.00001);
    BOOST_TEST(slamService.path_to_data == "/path/to/data");
    BOOST_TEST(slamService.path_to_map == "/path/to/map");
    BOOST_TEST(slamService.config_params == config_param);
    BOOST_TEST(slamService.slam_mode == "2d");
    BOOST_TEST(slamService.optimize_every_n_nodes == 9000);
    BOOST_TEST(slamService.num_range_data == 9001);
    BOOST_TEST(slamService.missing_data_ray_length == 9002.2, tolerance);
    BOOST_TEST(slamService.max_range == 9003.3, tolerance);
    BOOST_TEST(slamService.min_range == 9004.4, tolerance);
    BOOST_TEST(slamService.max_submaps_to_keep == 9005);  // LOCALIZATION only
    BOOST_TEST(slamService.fresh_submaps_count == 9006);  // UPDATING only
    BOOST_TEST(slamService.min_covered_area == 9007.7,
               tolerance);                                    // UPDATING only
    BOOST_TEST(slamService.min_added_submaps_count == 9008);  // UPDATING only
    BOOST_TEST(slamService.occupied_space_weight == 9009.9, tolerance);
    BOOST_TEST(slamService.translation_weight == 9010.1, tolerance);
    BOOST_TEST(slamService.rotation_weight == 9011.1, tolerance);
    BOOST_TEST(slamService.port == "localhost:0");
    BOOST_TEST(slamService.data_rate_ms.count() ==
               std::chrono::milliseconds(200).count());
    BOOST_TEST(slamService.map_rate_sec.count() ==
               std::chrono::seconds(60).count());
    BOOST_TEST(slamService.camera_name == "lidar");
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.optimize_on_start == true);
    BOOST_TEST(slamService.delete_processed_data == false);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_capitalized_slam_mode) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2D}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-data_rate_ms=200",
        "-map_rate_sec=60",   "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.slam_mode == "2d");
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_no_map_rate_sec) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server",
                                  "-config_param={mode=2d}",
                                  "-data_dir=/path/to",
                                  "-port=localhost:0",
                                  "-sensors=lidar",
                                  "-data_rate_ms=200",
                                  "-delete_processed_data=false",
                                  "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.map_rate_sec.count() ==
               std::chrono::seconds(60).count());
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_config_no_data_rate_ms) {
    ResetFlagsForTesting();
    std::vector<std::string> args{"carto_grpc_server",
                                  "-config_param={mode=2d}",
                                  "-data_dir=/path/to",
                                  "-port=localhost:0",
                                  "-sensors=lidar",
                                  "-map_rate_sec=60",
                                  "-delete_processed_data=false",
                                  "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.data_rate_ms.count() ==
               std::chrono::milliseconds(200).count());
    delete argv;
}

BOOST_AUTO_TEST_CASE(ParseAndValidateConfigParams_valid_config_no_camera) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",   "-config_param={mode=2d}",
        "-data_dir=/path/to",  "-port=localhost:0",
        "-sensors=",           "-data_rate_ms=200",
        "-map_rate_sec=60",    "-delete_processed_data=false",
        "-use_live_data=false"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.camera_name == "");
    BOOST_TEST(slamService.use_live_data == false);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_online_config_with_true_delete_processed_data) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2d}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-map_rate_sec=60",
        "-data_rate_ms=200",  "-delete_processed_data=true",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.delete_processed_data == true);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_online_config_with_false_delete_processed_data) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2d}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-map_rate_sec=60",
        "-data_rate_ms=200",  "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.use_live_data == true);
    BOOST_TEST(slamService.delete_processed_data == false);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_invalid_offline_config_with_true_delete_processed_data) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",   "-config_param={mode=2d}",
        "-data_dir=/path/to",  "-port=localhost:0",
        "-sensors=",           "-map_rate_sec=60",
        "-data_rate_ms=200",   "-delete_processed_data=true",
        "-use_live_data=false"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    const std::string message =
        "a true delete_processed_data value is invalid when running slam in "
        "offline mode";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_valid_offline_config_with_false_delete_processed_data) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",   "-config_param={mode=2d}",
        "-data_dir=/path/to",  "-port=localhost:0",
        "-sensors=",           "-map_rate_sec=60",
        "-data_rate_ms=200",   "-delete_processed_data=false",
        "-use_live_data=false"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.use_live_data == false);
    BOOST_TEST(slamService.delete_processed_data == false);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_config_with_true_use_live_data_and_sensors) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2d}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=lidar",     "-map_rate_sec=60",
        "-data_rate_ms=200",  "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.use_live_data == true);
    delete argv;
}

// TODO: Add test in once integration tests have been updated (See associated
// JIRA ticket: https://viam.atlassian.net/browse/RSDK-1625)
// BOOST_AUTO_TEST_CASE(
//     ParseAndValidateConfigParams_config_with_false_use_live_data_and_sensors)
//     { ResetFlagsForTesting(); std::vector<std::string> args{
//         "carto_grpc_server",  "-config_param={mode=2d}",
//         "-data_dir=/path/to", "-port=localhost:0",
//         "-sensors=lidar",     "-map_rate_sec=60",
//         "-data_rate_ms=200",  "-delete_processed_data=false",
//         "-use_live_data=false"};
//     int argc = args.size();
//     char** argv = toCharArrayArray(args);
//     SLAMServiceImpl slamService;
//     ParseAndValidateConfigParams(argc, argv, slamService);
//     BOOST_TEST(slamService.use_live_data == false);
//     delete argv;
// }

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_config_with_true_use_live_data_and_no_sensors) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",  "-config_param={mode=2d}",
        "-data_dir=/path/to", "-port=localhost:0",
        "-sensors=",          "-map_rate_sec=60",
        "-data_rate_ms=200",  "-delete_processed_data=false",
        "-use_live_data=true"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    const std::string message =
        "a true use_live_data value is invalid when no sensors are given";
    checkParseAndValidateConfigParamsException(argc, argv, message);
    delete argv;
}

BOOST_AUTO_TEST_CASE(
    ParseAndValidateConfigParams_config_with_false_use_live_data_and_no_sensors) {
    ResetFlagsForTesting();
    std::vector<std::string> args{
        "carto_grpc_server",   "-config_param={mode=2d}",
        "-data_dir=/path/to",  "-port=localhost:0",
        "-sensors=",           "-map_rate_sec=60",
        "-data_rate_ms=200",   "-delete_processed_data=false",
        "-use_live_data=false"};
    int argc = args.size();
    char** argv = toCharArrayArray(args);
    SLAMServiceImpl slamService;
    ParseAndValidateConfigParams(argc, argv, &slamService);
    BOOST_TEST(slamService.use_live_data == false);
    delete argv;
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace
}  // namespace config
}  // namespace viam
