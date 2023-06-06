// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

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

void validate_mode(viam_carto_MODE mode) {
    switch (mode) {
        case VIAM_CARTO_LOCALIZING:
            break;
        case VIAM_CARTO_MAPPING:
            break;
        case VIAM_CARTO_UPDATING:
            break;
        default:
            throw VIAM_CARTO_SLAM_MODE_INVALID;
    }
}

void validate_lidar_config(viam_carto_LIDAR_CONFIG lidar_config) {
    switch (lidar_config) {
        case VIAM_CARTO_TWO_D:
            break;
        case VIAM_CARTO_THREE_D:
            break;
        default:
            throw VIAM_CARTO_LIDAR_CONFIG_INVALID;
    }
}

config from_viam_carto_config(viam_carto_config vcc) {
    struct config c;
    for (int i = 0; i < vcc.sensors_len; i++) {
        c.sensors.push_back(std_string_from_bstring(vcc.sensors[i]));
    }

    c.data_dir = std_string_from_bstring(vcc.data_dir);
    c.component_reference = std_string_from_bstring(vcc.component_reference);
    c.map_rate_sec = std::chrono::seconds(vcc.map_rate_sec);
    c.mode = vcc.mode;
    c.lidar_config = vcc.lidar_config;
    if (c.sensors.size() == 0) {
        throw VIAM_CARTO_SENSORS_LIST_EMPTY;
    }
    if (c.data_dir.size() == 0) {
        throw VIAM_CARTO_DATA_DIR_NOT_PROVIDED;
    }
    if (vcc.map_rate_sec < 0) {
        throw VIAM_CARTO_MAP_RATE_SEC_INVALID;
    }
    if (c.component_reference.empty()) {
        throw VIAM_CARTO_COMPONENT_REFERENCE_INVALID;
    }
    validate_mode(c.mode);
    validate_lidar_config(c.lidar_config);

    return c;
};

std::string find_lua_files() {
    std::string configuration_directory;
    auto programLocation = boost::dll::program_location();
    auto relativePathToLuas = programLocation.parent_path().parent_path();
    relativePathToLuas.append("share/cartographer/lua_files");
    boost::filesystem::path absolutePathToLuas(
        "/usr/local/share/cartographer/lua_files");
    if (exists(relativePathToLuas)) {
        VLOG(1) << "Using lua files from relative path";
        configuration_directory = relativePathToLuas.string();
    } else if (exists(absolutePathToLuas)) {
        VLOG(1) << "Using lua files from absolute path";
        configuration_directory = absolutePathToLuas.string();
    } else {
        LOG(ERROR) << "No lua files found, looked in " << relativePathToLuas;
        LOG(ERROR) << "Use 'make install-lua-files' to install lua files into "
                      "/usr/local/share";
    }
    return configuration_directory;
};

const std::string action_mode_lua_config_filename(ActionMode am) {
    switch (am) {
        case ActionMode::MAPPING:
            return configuration_mapping_basename;
            break;
        case ActionMode::LOCALIZING:
            return configuration_localization_basename;
            break;
        case ActionMode::UPDATING:
            return configuration_update_basename;
            break;
        default:
            throw VIAM_CARTO_SLAM_MODE_INVALID;
    }
};

CartoFacade::CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                         const viam_carto_algo_config ac) {
    lib = pVCL;
    config = from_viam_carto_config(c);
    algo_config = ac;
    path_to_data = config.data_dir + "/data";
    // TODO: Change to "/internal_state"
    path_to_internal_state = config.data_dir + "/map";
    b_continue_session = true;
};

int CartoFacade::IOInit() {
    auto cd = find_lua_files();
    if (cd.empty()) {
        return VIAM_CARTO_LUA_CONFIG_NOT_FOUND;
    }
    configuration_directory = cd;

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        // Set TrajectoryBuilder
        trajectory_id = map_builder.SetTrajectoryBuilder(&trajectory_builder,
                                                         {kRangeSensorId});
        VLOG(1) << "Using trajectory ID: " << trajectory_id;
    }

    // Setting the action mode has to happen before setting up the
    // map builder.
    /* action_mode = viam::utils::determine_action_mode(path_to_internal_state,
     */
    /*                                                  config.map_rate_sec); */
    /* auto config_basename = action_mode_lua_config_filename(action_mode); */
    /* { */
    /*     std::lock_guard<std::mutex> lk(map_builder_mutex); */
    /*     map_builder.SetUp(configuration_directory, config_basename); */

    /* map_builder.OverwriteOptimizeEveryNNodes( */
    /*     algo_config.optimize_every_n_nodes); */
    /* map_builder.OverwriteNumRangeData(algo_config.num_range_data); */
    /* map_builder.OverwriteMissingDataRayLength( */
    /*     algo_config.missing_data_ray_length); */
    /* map_builder.OverwriteMaxRange(algo_config.max_range); */
    /* map_builder.OverwriteMinRange(algo_config.min_range); */
    /* if (action_mode == ActionMode::LOCALIZING) { */
    /*     map_builder.OverwriteMaxSubmapsToKeep(algo_config.max_submaps_to_keep);
     */
    /* } */
    /* if (action_mode == ActionMode::UPDATING) { */
    /*     map_builder.OverwriteFreshSubmapsCount(algo_config.fresh_submaps_count);
     */
    /*     map_builder.OverwriteMinCoveredArea(algo_config.min_covered_area); */
    /*     map_builder.OverwriteMinAddedSubmapsCount( */
    /*         algo_config.min_added_submaps_count); */
    /* } */
    /* map_builder.OverwriteOccupiedSpaceWeight(algo_config.occupied_space_weight);
     */
    /* map_builder.OverwriteTranslationWeight(algo_config.translation_weight);
     */
    /* map_builder.OverwriteRotationWeight(algo_config.rotation_weight); */

    /*     map_builder.BuildMapBuilder(); */
    /* } */

    return VIAM_CARTO_SUCCESS;
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
std::ostream &operator<<(std::ostream &os,
                         const viam::carto_facade::ActionMode &action_mode) {
    std::string action_mode_str;
    if (action_mode == viam::carto_facade::ActionMode::MAPPING) {
        action_mode_str = "mapping";
    } else if (action_mode == viam::carto_facade::ActionMode::LOCALIZING) {
        action_mode_str = "localizing";
    } else if (action_mode == viam::carto_facade::ActionMode::UPDATING) {
        action_mode_str = "updating";
    } else {
        throw std::runtime_error("invalid ActionMode value");
    }
    os << action_mode_str;
    return os;
}

std::vector<std::string> list_sorted_files_in_directory(
    std::string data_directory) {
    std::vector<std::string> file_paths;

    for (const auto &entry :
         boost::filesystem::directory_iterator(data_directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

viam::carto_facade::ActionMode determine_action_mode(
    std::string path_to_map, std::chrono::seconds map_rate_sec) {
    // Check if there is an apriori map in the path_to_map directory
    std::vector<std::string> map_filenames =
        list_sorted_files_in_directory(path_to_map);

    // Check if there is a *.pbstream map in the path_to_map directory
    for (auto filename : map_filenames) {
        if (filename.find(".pbstream") != std::string::npos) {
            // There is an apriori map present, so we're running either in
            // updating or localization mode.
            if (map_rate_sec.count() == 0) {
                // This log line is needed by rdk integration tests.
                LOG(INFO) << "Running in localization only mode";
                return viam::carto_facade::ActionMode::LOCALIZING;
            }
            // This log line is needed by rdk integration tests.
            LOG(INFO) << "Running in updating mode";
            return viam::carto_facade::ActionMode::UPDATING;
        }
    }
    if (map_rate_sec.count() == 0) {
        throw std::runtime_error(
            "set to localization mode (map_rate_sec = 0) but couldn't find "
            "apriori map to localize on");
    }
    // This log line is needed by rdk integration tests.
    LOG(INFO) << "Running in mapping mode";
    return viam::carto_facade::ActionMode::MAPPING;
}

std::string get_latest_map_filename(std::string path_to_map) {
    std::string latest_map_filename;

    std::vector<std::string> map_filenames =
        list_sorted_files_in_directory(path_to_map);
    bool found_map = false;
    for (int i = map_filenames.size() - 1; i >= 0; i--) {
        if (map_filenames.at(i).find(".pbstream") != std::string::npos) {
            latest_map_filename = map_filenames.at(i);
            found_map = true;
            break;
        }
    }
    if (!found_map) {
        throw std::runtime_error("cannot find maps but they should be present");
    }

    return latest_map_filename;
}

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

    // allocate viam_carto struct
    viam_carto *vc = (viam_carto *)malloc(sizeof(viam_carto));
    if (vc == nullptr) {
        return VIAM_CARTO_OUT_OF_MEMORY;
    }

    try {
        vc->carto_obj = new viam::carto_facade::CartoFacade(pVCL, c, ac);
    } catch (int err) {
        free(vc);
        return err;
    } catch (std::exception &e) {
        free(vc);
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }

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
