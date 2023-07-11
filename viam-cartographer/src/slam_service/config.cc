// This is an experimental integration of cartographer into RDK.

#include "config.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

#include "../utils/slam_service_helpers.h"
#include "glog/logging.h"

using namespace boost::filesystem;
namespace viam {
namespace config {

int defaultIMUDataRateMS = 20;
int defaultDataRateMS = 200;
int defaultMapRateSec = 60;

DEFINE_string(data_dir, "",
              "Directory in which sensor data and maps are expected.");
DEFINE_string(config_param, "", "Config parameters for cartographer.");
DEFINE_string(port, "", "GRPC port");
DEFINE_string(sensors, "", "Array of sensors.");
DEFINE_int64(data_rate_ms, defaultDataRateMS,
             "Frequency at which we grab/save data.");
DEFINE_int64(imu_data_rate_ms, defaultIMUDataRateMS,
             "Frequency at which we grab/save IMU data.");
DEFINE_int64(
    map_rate_sec, defaultMapRateSec,
    "Frequency at which we want to print map pictures while cartographer "
    "is running.");
DEFINE_bool(delete_processed_data, false,
            "Deletes data after it has been processed");
DEFINE_bool(use_live_data, false,
            "Indicate whether or not SLAM should use new live-generated data "
            "or previously generated data");
DEFINE_bool(aix_auto_update, false, "Automatically updates the app image");

void ParseAndValidateConfigParams(int argc, char** argv,
                                  SLAMServiceImpl& slamService) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_config_param.empty()) {
        throw std::runtime_error("-config_param is missing");
    }
    const auto minloglevel =
        ConfigParamParser(FLAGS_config_param, "minloglevel=");
    if (!minloglevel.empty()) {
        FLAGS_minloglevel = std::stoi(minloglevel);
    }
    const auto v = ConfigParamParser(FLAGS_config_param, "v=");
    if (!v.empty()) {
        FLAGS_v = std::stoi(v);
    }

    if (FLAGS_data_dir.empty()) {
        throw std::runtime_error("-data_dir is missing");
    }
    if (FLAGS_port.empty()) {
        throw std::runtime_error("-port is missing");
    }

    LOG(INFO) << "data_dir: " << FLAGS_data_dir;
    LOG(INFO) << "config_param: " << FLAGS_config_param;
    LOG(INFO) << "port: " << FLAGS_port;
    LOG(INFO) << "sensors: " << FLAGS_sensors;
    LOG(INFO) << "data_rate_ms: " << FLAGS_data_rate_ms;
    LOG(INFO) << "imu_data_rate_ms: " << FLAGS_imu_data_rate_ms;
    LOG(INFO) << "map_rate_sec: " << FLAGS_map_rate_sec;
    LOG(INFO) << "delete_processed_data: " << FLAGS_delete_processed_data;
    LOG(INFO) << "use_live_data: " << FLAGS_use_live_data;

    slamService.path_to_data = FLAGS_data_dir + "/data";
    slamService.path_to_map = FLAGS_data_dir + "/map";
    slamService.use_live_data = FLAGS_use_live_data;
    if (slamService.use_live_data && FLAGS_sensors.empty()) {
        throw std::runtime_error(
            "a true use_live_data value is invalid when no sensors are given");
    }

    // Find the lua files.
    auto programLocation = boost::dll::program_location();
    auto relativePathToLuas = programLocation.parent_path().parent_path();
    relativePathToLuas.append("share/cartographer/lua_files");
    boost::filesystem::path absolutePathToLuas(
        "/usr/local/share/cartographer/lua_files");
    if (exists(relativePathToLuas)) {
        VLOG(1) << "Using lua files from relative path";
        slamService.configuration_directory = relativePathToLuas.string();
    } else if (exists(absolutePathToLuas)) {
        VLOG(1) << "Using lua files from absolute path";
        slamService.configuration_directory = absolutePathToLuas.string();
    } else {
        LOG(ERROR) << "No lua files found, looked in " << relativePathToLuas;
        LOG(ERROR) << "Use 'make install-lua-files' to install lua files into "
                      "/usr/local/share";
    }

    slamService.config_params = FLAGS_config_param;
    slamService.port = FLAGS_port;
    slamService.camera_name = FLAGS_sensors;
    slamService.data_rate_ms = std::chrono::milliseconds(FLAGS_data_rate_ms);
    slamService.imu_data_rate_ms = std::chrono::milliseconds(FLAGS_imu_data_rate_ms);
    slamService.map_rate_sec = std::chrono::seconds(FLAGS_map_rate_sec);

    slamService.delete_processed_data = FLAGS_delete_processed_data;
    if (!slamService.use_live_data && slamService.delete_processed_data) {
        throw std::runtime_error(
            "a true delete_processed_data value is invalid when running slam "
            "in offline mode");
    }

    slamService.slam_mode =
        ConfigParamParser(slamService.config_params, "mode=");
    if (slamService.slam_mode.empty()) {
        throw std::runtime_error("slam mode is missing");
    }

    boost::algorithm::to_lower(slamService.slam_mode);
    if (slamService.slam_mode != "2d" && slamService.slam_mode != "3d") {
        throw std::runtime_error("Invalid slam_mode=" + slamService.slam_mode);
    }

    const auto optimize_on_start =
        ConfigParamParser(FLAGS_config_param, "optimize_on_start=");
    if (optimize_on_start == "true") {
        slamService.optimize_on_start = true;
    }

    std::vector<std::string> carto_params = {"optimize_every_n_nodes",
                                             "num_range_data",
                                             "missing_data_ray_length",
                                             "max_range",
                                             "min_range",
                                             "max_submaps_to_keep",
                                             "fresh_submaps_count",
                                             "min_covered_area",
                                             "min_added_submaps_count",
                                             "occupied_space_weight",
                                             "translation_weight",
                                             "rotation_weight"};
    for (auto&& carto_param : carto_params)
        OverwriteCartoConfigParam(slamService, carto_param);
}

void OverwriteCartoConfigParam(SLAMServiceImpl& slamService,
                               const std::string& parameter) {
    std::string new_parameter =
        ConfigParamParser(slamService.config_params, parameter + "=");

    ActionMode slam_action_mode = slamService.GetActionMode();
    if (!new_parameter.empty()) {
        LOG(INFO) << parameter << " is overwritten to: " << new_parameter;

        if (parameter == "optimize_every_n_nodes") {
            slamService.optimize_every_n_nodes = std::stoi(new_parameter);
        } else if (parameter == "num_range_data") {
            slamService.num_range_data = std::stoi(new_parameter);
        } else if (parameter == "missing_data_ray_length") {
            slamService.missing_data_ray_length = std::stof(new_parameter);
        } else if (parameter == "max_range") {
            slamService.max_range = std::stof(new_parameter);
        } else if (parameter == "min_range") {
            slamService.min_range = std::stof(new_parameter);
        } else if (parameter == "max_submaps_to_keep") {
            if (slam_action_mode != ActionMode::LOCALIZING) {
                LOG(WARNING) << "Not in localizing action mode: Setting "
                                "max_submaps_to_keep has no effect";
            }
            slamService.max_submaps_to_keep = std::stoi(new_parameter);
        } else if (parameter == "fresh_submaps_count") {
            if (slam_action_mode != ActionMode::UPDATING) {
                LOG(WARNING) << "Not in updating action mode: Setting "
                                "fresh_submaps_count has no effect";
            }
            slamService.fresh_submaps_count = std::stoi(new_parameter);
        } else if (parameter == "min_covered_area") {
            if (slam_action_mode != ActionMode::UPDATING) {
                LOG(WARNING) << "Not in updating action mode: Setting "
                                "min_covered_area has no effect";
            }
            slamService.min_covered_area = std::stod(new_parameter);
        } else if (parameter == "min_added_submaps_count") {
            if (slam_action_mode != ActionMode::UPDATING) {
                LOG(WARNING) << "Not in updating action mode: Setting "
                                "min_added_submaps_count has no effect";
            }
            slamService.min_added_submaps_count = std::stoi(new_parameter);
        } else if (parameter == "occupied_space_weight") {
            slamService.occupied_space_weight = std::stod(new_parameter);
        } else if (parameter == "translation_weight") {
            slamService.translation_weight = std::stod(new_parameter);
        } else if (parameter == "rotation_weight") {
            slamService.rotation_weight = std::stod(new_parameter);
        } else {
            throw std::runtime_error(
                "unsupported cartographer config parameter: " + parameter);
        }
    }
}

// Parse a config parameter map for a specific variable name and return the
// value as a string. Returns empty if the variable is not found within the map.
std::string ConfigParamParser(std::string map, std::string varName) {
    std::string strVal;
    size_t loc = std::string::npos;

    std::stringstream ss(map.substr(map.find("{") + 1, map.find("}") - 1));
    while (ss.good()) {
        std::string substr;
        getline(ss, substr, ',');
        loc = substr.find(varName);
        if (loc != std::string::npos) {
            strVal = substr.substr(loc + varName.size());
            break;
        }
    }

    return strVal;
}

void ResetFlagsForTesting() {
    FLAGS_config_param = "";
    FLAGS_data_dir = "";
    FLAGS_port = "";
    FLAGS_sensors = "";
    FLAGS_data_rate_ms = defaultDataRateMS;
    FLAGS_imu_data_rate_ms = defaultIMUDataRateMS;
    FLAGS_map_rate_sec = defaultMapRateSec;
    FLAGS_delete_processed_data = false;
    FLAGS_use_live_data = false;
}

}  // namespace config
}  // namespace viam
