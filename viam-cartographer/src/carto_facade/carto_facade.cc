// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "glog/logging.h"
#include "io.h"
#include "map_builder.h"
#include "util.h"

namespace viam {
namespace carto_facade {
namespace fs = boost::filesystem;
// Number of bytes in a pixel
static const int bytesPerPixel = 4;
struct ColorARGB {
    unsigned char A;
    unsigned char R;
    unsigned char G;
    unsigned char B;
};

// Check if the green color channel is 0 to filter unobserved pixels which is
// set in DrawTexture at
// https://github.com/cartographer-project/cartographer/blob/ef00de231
// 7dcf7895b09f18cc4d87f8b533a019b/cartographer/io/submap_painter.cc#L206-L207
bool check_if_empty_pixel(ColorARGB pixel_color) {
    return (pixel_color.G == 0);
}

std::string to_std_string(bstring b_str) {
    int len = blength(b_str);
    char *tmp = bstr2cstr(b_str, 0);
    if (tmp == NULL) {
        throw VIAM_CARTO_OUT_OF_MEMORY;
    }
    std::string std_str(tmp, len);
    bcstrfree(tmp);
    return std_str;
}

bstring to_bstring(std::string str) {
    bstring bstr = blk2bstr(str.c_str(), str.length());
    if (bstr == NULL) {
        throw VIAM_CARTO_OUT_OF_MEMORY;
    }
    return bstr;
}

// Convert the scale of a specified color channel from the given UCHAR
// range of 0 - 255 to an inverse probability range of 100 - 0.
int calculate_probability_from_color_channels(ColorARGB pixel_color) {
    unsigned char max_val = UCHAR_MAX;
    unsigned char min_val = 0;
    unsigned char max_prob = 100;
    unsigned char min_prob = 0;

    // Probability is currently determined solely by the red color channel.
    unsigned char color_channel_val = pixel_color.R;
    unsigned char prob = (max_val - color_channel_val) * (max_prob - min_prob) /
                         (max_val - min_val);
    return prob;
}

std::ostream &operator<<(std::ostream &os, const ActionMode &action_mode) {
    std::string action_mode_str;
    if (action_mode == ActionMode::MAPPING) {
        action_mode_str = "mapping";
    } else if (action_mode == ActionMode::LOCALIZING) {
        action_mode_str = "localizing";
    } else if (action_mode == ActionMode::UPDATING) {
        action_mode_str = "updating";
    } else {
        throw std::runtime_error("invalid ActionMode value");
    }
    os << action_mode_str;
    return os;
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
        c.sensors.push_back(to_std_string(vcc.sensors[i]));
    }
    c.data_dir = to_std_string(vcc.data_dir);
    c.map_rate_sec = std::chrono::seconds(vcc.map_rate_sec);
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
    if (c.sensors[0].empty()) {
        throw VIAM_CARTO_COMPONENT_REFERENCE_INVALID;
    }
    validate_lidar_config(c.lidar_config);
    c.component_reference = bstrcpy(vcc.sensors[0]);

    return c;
};

std::string find_lua_files() {
    auto programLocation = boost::dll::program_location();
    auto localRelativePathToLuas = programLocation.parent_path().parent_path();
    localRelativePathToLuas.append("lua_files");
    auto relativePathToLuas = programLocation.parent_path().parent_path();
    relativePathToLuas.append("share/cartographer/lua_files");
    fs::path absolutePathToLuas("/usr/local/share/cartographer/lua_files");

    if (exists(relativePathToLuas)) {
        VLOG(1) << "Using lua files from relative path "
                << relativePathToLuas.string();
        return relativePathToLuas.string();
    } else if (exists(localRelativePathToLuas)) {
        VLOG(1) << "Using lua files from local relative path "
                << localRelativePathToLuas.string();
        return localRelativePathToLuas.string();
    } else if (exists(absolutePathToLuas)) {
        VLOG(1) << "Using lua files from absolute path "
                << absolutePathToLuas.string();
        return absolutePathToLuas.string();
    } else {
        LOG(ERROR) << "No lua files found, looked in " << relativePathToLuas;
        LOG(ERROR) << "Use 'make install-lua-files' to install lua files into "
                      "/usr/local/share";
        return "";
    }
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
            LOG(ERROR)
                << "action_mode_lua_config_filename: action mode is invalid";
            throw VIAM_CARTO_SLAM_MODE_INVALID;
    }
};

CartoFacade::CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                         const viam_carto_algo_config ac) {
    lib = pVCL;
    config = from_viam_carto_config(c);
    algo_config = ac;
    path_to_internal_state = config.data_dir + "/internal_state";
};

CartoFacade::~CartoFacade() { bdestroy(config.component_reference); }

void setup_filesystem(std::string data_dir,
                      std::string path_to_internal_state) {
    // setup internal state directory if it doesn't exist
    auto perms =
        fs::perms::group_all | fs::perms::owner_all | fs::perms::others_read;
    try {
        // if directory doesn't exist, create it with expected structure
        if (!fs::is_directory(data_dir)) {
            VLOG(1) << "data_dir doesn't exist. Creating " << data_dir;
            fs::create_directory(data_dir);
            VLOG(1) << "setting data_dir permissions";
            fs::permissions(data_dir, perms);
        }
        if (!fs::is_directory(path_to_internal_state)) {
            VLOG(1) << "data_dir's internal_state directory, doesn't exist. "
                       "Creating "
                    << path_to_internal_state;
            fs::create_directory(path_to_internal_state);
            VLOG(1)
                << "setting data_dir's internal_state directory's permissions";
            fs::permissions(path_to_internal_state, perms);
        }
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        throw VIAM_CARTO_DATA_DIR_FILE_SYSTEM_ERROR;
    }
}

std::vector<std::string> list_sorted_files_in_directory(std::string directory) {
    std::vector<std::string> file_paths;

    for (const auto &entry : fs::directory_iterator(directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

std::string get_latest_internal_state_filename(
    std::string path_to_internal_state) {
    std::string latest_internal_state_filename;

    std::vector<std::string> internal_state_filename =
        list_sorted_files_in_directory(path_to_internal_state);
    bool found_internal_state = false;
    for (int i = internal_state_filename.size() - 1; i >= 0; i--) {
        if (internal_state_filename.at(i).find(".pbstream") !=
            std::string::npos) {
            latest_internal_state_filename = internal_state_filename.at(i);
            found_internal_state = true;
            break;
        }
    }
    if (!found_internal_state) {
        throw std::runtime_error(
            "cannot find internal state but they should be present");
    }

    return latest_internal_state_filename;
}

void CartoFacade::IOInit() {
    // Detect if data_dir has deprecated format
    if (fs::is_directory(config.data_dir + "/data")) {
        LOG(ERROR) << "data directory " << config.data_dir
                   << " is invalid as it contains deprecated format i.e. /data "
                      "subdirectory";
        throw VIAM_CARTO_DATA_DIR_INVALID_DEPRECATED_STRUCTURE;
    }
    // Setup file system for saving internal state
    setup_filesystem(config.data_dir, path_to_internal_state);
    action_mode =
        determine_action_mode(path_to_internal_state, config.map_rate_sec);
    VLOG(1) << "slam action mode: " << action_mode;
    // TODO: Make this API user configurable
    auto cd = find_lua_files();
    if (cd.empty()) {
        throw VIAM_CARTO_LUA_CONFIG_NOT_FOUND;
    }
    configuration_directory = cd;
    // Detect action mode
    auto config_basename = action_mode_lua_config_filename(action_mode);
    // Setup MapBuilder
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory, config_basename);
        VLOG(1) << "overwriting map_builder config";
        map_builder.OverwriteOptimizeEveryNNodes(
            algo_config.optimize_every_n_nodes);
        map_builder.OverwriteNumRangeData(algo_config.num_range_data);
        map_builder.OverwriteMissingDataRayLength(
            algo_config.missing_data_ray_length);
        map_builder.OverwriteMaxRange(algo_config.max_range);
        map_builder.OverwriteMinRange(algo_config.min_range);
        if (action_mode == ActionMode::LOCALIZING) {
            map_builder.OverwriteMaxSubmapsToKeep(
                algo_config.max_submaps_to_keep);
        }
        if (action_mode == ActionMode::UPDATING) {
            map_builder.OverwriteFreshSubmapsCount(
                algo_config.fresh_submaps_count);
            map_builder.OverwriteMinCoveredArea(algo_config.min_covered_area);
            map_builder.OverwriteMinAddedSubmapsCount(
                algo_config.min_added_submaps_count);
        }
        map_builder.OverwriteOccupiedSpaceWeight(
            algo_config.occupied_space_weight);
        map_builder.OverwriteTranslationWeight(algo_config.translation_weight);
        map_builder.OverwriteRotationWeight(algo_config.rotation_weight);
        map_builder.BuildMapBuilder();
    }

    // TODO: google cartographer will termiante the program if
    // the internal state is invalid
    // see https://viam.atlassian.net/browse/RSDK-3553
    if (action_mode == ActionMode::UPDATING ||
        action_mode == ActionMode::LOCALIZING) {
        // Check if there is an apriori map (internal state) in the
        // path_to_internal_state directory
        std::string latest_internal_state_filename =
            get_latest_internal_state_filename(path_to_internal_state);
        VLOG(1) << "latest_internal_state_filename: "
                << latest_internal_state_filename;
        // load_frozen_trajectory has to be true for LOCALIZING action mode,
        // and false for UPDATING action mode.
        bool load_frozen_trajectory = (action_mode == ActionMode::LOCALIZING);
        if (algo_config.optimize_on_start) {
            VLOG(1) << "running optimize_on_start";
            CacheLatestMap();

            std::unique_lock optimization_lock{optimization_shared_mutex,
                                               std::defer_lock};
            optimization_lock.lock();
            // Load apriori map (internal state)
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.LoadMapFromFile(latest_internal_state_filename,
                                        load_frozen_trajectory,
                                        algo_config.optimize_on_start);
        } else {
            // Load apriori map (internal state)
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.LoadMapFromFile(latest_internal_state_filename,
                                        load_frozen_trajectory,
                                        algo_config.optimize_on_start);
        }

        CacheMapInLocalizationMode();
    }

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.StartLidarTrajectoryBuilder();
    }
};

void CartoFacade::CacheLatestMap() {
    VLOG(1) << "CacheLatestMap()";
    std::string pointcloud_map_tmp;
    try {
        GetLatestSampledPointCloudMapString(pointcloud_map_tmp);

    } catch (std::exception &e) {
        LOG(ERROR) << "error encoding pointcloud map: " << e.what();
        throw VIAM_CARTO_MAP_CREATION_ERROR;
    }
    std::lock_guard<std::mutex> lk(viam_response_mutex);
    latest_pointcloud_map = std::move(pointcloud_map_tmp);
}

// If using the LOCALIZING action mode, cache a copy of the map before
// beginning to process data. If cartographer fails to do this,
// terminate the program
void CartoFacade::CacheMapInLocalizationMode() {
    VLOG(1) << "CacheMapInLocalizationMode()";
    if (action_mode == ActionMode::LOCALIZING) {
        std::string pointcloud_map_tmp;
        try {
            GetLatestSampledPointCloudMapString(pointcloud_map_tmp);

        } catch (std::exception &e) {
            LOG(ERROR) << "error encoding localized "
                          "pointcloud map: "
                       << e.what();
            throw VIAM_CARTO_MAP_CREATION_ERROR;
        }

        if (pointcloud_map_tmp.empty()) {
            LOG(ERROR) << "error encoding localized "
                          "pointcloud map: no map points";
            throw VIAM_CARTO_MAP_CREATION_ERROR;
        }

        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_pointcloud_map = std::move(pointcloud_map_tmp);
        }
    }
}

cartographer::io::PaintSubmapSlicesResult
CartoFacade::GetLatestPaintedMapSlices() {
    VLOG(1) << "GetLatestPaintedMapSlices()";
    cartographer::mapping::MapById<
        cartographer::mapping::SubmapId,
        cartographer::mapping::PoseGraphInterface::SubmapPose>
        submap_poses;
    std::map<cartographer::mapping::SubmapId,
             cartographer::mapping::proto::SubmapQuery::Response>
        response_protos;

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        submap_poses =
            map_builder.map_builder_->pose_graph()->GetAllSubmapPoses();

        for (const auto &&submap_id_pose : submap_poses) {
            cartographer::mapping::proto::SubmapQuery::Response
                &response_proto = response_protos[submap_id_pose.id];
            const std::string error = map_builder.map_builder_->SubmapToProto(
                submap_id_pose.id, &response_proto);
            if (error != "") {
                throw std::runtime_error(error);
            }
        }
    }

    std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;

    if (submap_poses.size() == 0) {
        throw std::runtime_error(viam::carto_facade::errorNoSubmaps);
    }

    for (const auto &&submap_id_pose : submap_poses) {
        auto submap_textures =
            absl::make_unique<::cartographer::io::SubmapTextures>();
        submap_textures->version =
            response_protos[submap_id_pose.id].submap_version();
        for (const auto &texture_proto :
             response_protos[submap_id_pose.id].textures()) {
            const std::string compressed_cells(texture_proto.cells().begin(),
                                               texture_proto.cells().end());
            submap_textures->textures.emplace_back(
                ::cartographer::io::SubmapTexture{
                    ::cartographer::io::UnpackTextureData(
                        compressed_cells, texture_proto.width(),
                        texture_proto.height()),
                    texture_proto.width(), texture_proto.height(),
                    texture_proto.resolution(),
                    cartographer::transform::ToRigid3(
                        texture_proto.slice_pose())});
        }

        // Prepares SubmapSlice
        ::cartographer::io::SubmapSlice &submap_slice =
            submap_slices[submap_id_pose.id];
        const auto fetched_texture = submap_textures->textures.begin();
        submap_slice.pose = submap_id_pose.data.pose;
        submap_slice.width = fetched_texture->width;
        submap_slice.height = fetched_texture->height;
        submap_slice.slice_pose = fetched_texture->slice_pose;
        submap_slice.resolution = fetched_texture->resolution;
        submap_slice.cairo_data.clear();

        submap_slice.surface = ::cartographer::io::DrawTexture(
            fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
            fetched_texture->width, fetched_texture->height,
            &submap_slice.cairo_data);
    }
    cartographer::io::PaintSubmapSlicesResult painted_slices =
        cartographer::io::PaintSubmapSlices(submap_slices, resolutionMeters);

    return painted_slices;
}

void CartoFacade::GetLatestSampledPointCloudMapString(std::string &pointcloud) {
    VLOG(1) << "GetLatestSampledPointCloudMapString()";
    std::unique_ptr<cartographer::io::PaintSubmapSlicesResult> painted_slices =
        nullptr;
    try {
        painted_slices =
            std::make_unique<cartographer::io::PaintSubmapSlicesResult>(
                GetLatestPaintedMapSlices());
    } catch (std::exception &e) {
        if (e.what() == viam::carto_facade::errorNoSubmaps) {
            VLOG(1) << "Error creating pcd map: " << e.what();
            LOG(INFO) << "Error creating pcd map: " << e.what();
            return;
        } else {
            std::string errorLog = "Error writing submap to proto: ";
            errorLog += e.what();
            LOG(ERROR) << errorLog;
            throw std::runtime_error(errorLog);
        }
    }

    // Get data from painted surface in ARGB32 format
    auto painted_surface = painted_slices->surface.get();
    auto image_format = cairo_image_surface_get_format(painted_surface);
    if (image_format != cartographer::io::kCairoFormat) {
        std::string error_log =
            "Error cairo surface in wrong format, expected Cairo_Format_ARGB32";
        LOG(ERROR) << error_log;
        throw std::runtime_error(error_log);
    }
    int width = cairo_image_surface_get_width(painted_surface);
    int height = cairo_image_surface_get_height(painted_surface);
    auto image_data_ptr = cairo_image_surface_get_data(painted_surface);

    // Get pixel containing map origin (0, 0)
    float origin_pixel_x = painted_slices->origin.x();
    float origin_pixel_y = painted_slices->origin.y();

    // Iterate over image data and add to pointcloud buffer
    int num_points = 0;
    std::string pcd_data;
    for (int pixel_y = 0; pixel_y < height; pixel_y++) {
        for (int pixel_x = 0; pixel_x < width; pixel_x++) {
            // Get byte index associated with pixel
            int pixel_index = pixel_x + pixel_y * width;
            int byte_index = pixel_index * bytesPerPixel;

            // We assume we are running on a little-endian system, so the ARGB
            // order is reversed
            ColorARGB pixel_color;
            pixel_color.A = image_data_ptr[byte_index + 3];
            pixel_color.R = image_data_ptr[byte_index + 2];
            pixel_color.G = image_data_ptr[byte_index + 1];
            pixel_color.B = image_data_ptr[byte_index + 0];

            // Skip pixel if it contains empty data (default color)
            if (check_if_empty_pixel(pixel_color)) {
                continue;
            }

            // Determine probability based on the color of the pixel and skip if
            // it is 0
            int prob = calculate_probability_from_color_channels(pixel_color);
            if (prob == 0) {
                continue;
            }

            // Convert pixel location to pointcloud point in meters
            float x_pos = (pixel_x - origin_pixel_x) * resolutionMeters;
            // Y is inverted to match output from getPosition()
            float y_pos = -(pixel_y - origin_pixel_y) * resolutionMeters;
            float z_pos = 0;  // Z is 0 in 2D SLAM

            // Add point to buffer
            viam::carto_facade::util::write_float_to_buffer_in_bytes(pcd_data,
                                                                     x_pos);
            viam::carto_facade::util::write_float_to_buffer_in_bytes(pcd_data,
                                                                     y_pos);
            viam::carto_facade::util::write_float_to_buffer_in_bytes(pcd_data,
                                                                     z_pos);
            viam::carto_facade::util::write_int_to_buffer_in_bytes(pcd_data,
                                                                   prob);

            num_points++;
        }
    }

    // Write our PCD file, which is written as a binary.
    pointcloud = viam::carto_facade::util::pcd_header(num_points, true);

    // Writes data buffer to the pointcloud string
    pointcloud += pcd_data;
    return;
}

void CartoFacade::GetPosition(viam_carto_get_position_response *r) {
    cartographer::transform::Rigid3d global_pose;
    {
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        VLOG(1) << "latest_global_pose: " << latest_global_pose;
        global_pose = latest_global_pose;
    }

    auto pos_vector = global_pose.translation();
    auto pos_quat = global_pose.rotation();

    r->x = pos_vector.x() * 1000;
    r->y = pos_vector.y() * 1000;
    r->z = pos_vector.z() * 1000;
    r->real = pos_quat.w();
    r->imag = pos_quat.x();
    r->jmag = pos_quat.y();
    r->kmag = pos_quat.z();
    r->component_reference = bstrcpy(config.component_reference);
};

void CartoFacade::GetPointCloudMap(viam_carto_get_point_cloud_map_response *r) {
    std::string pointcloud_map;
    // Write or grab the latest pointcloud map in form of a string
    std::shared_lock optimization_lock{optimization_shared_mutex,
                                       std::defer_lock};
    if (action_mode != ActionMode::LOCALIZING && optimization_lock.try_lock()) {
        // We are able to lock the optimization_shared_mutex, which means
        // that the optimization is not ongoing and we can grab the newest
        // map
        GetLatestSampledPointCloudMapString(pointcloud_map);
        VLOG(1) << "after GetLatestSampledPointCloudMapString()";
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        latest_pointcloud_map = pointcloud_map;
    } else {
        // Either we are in localization mode or we couldn't lock the mutex
        // which means the optimization process locked it and we need to use
        // the backed up latest map
        if (action_mode == ActionMode::LOCALIZING) {
            LOG(INFO) << "In localization mode, using cached pointcloud map";
        } else {
            LOG(INFO)
                << "Optimization is occuring, using cached pointcloud map";
        }
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        pointcloud_map = latest_pointcloud_map;
    }

    if (pointcloud_map.empty()) {
        LOG(ERROR) << "map pointcloud does not have points yet";
        throw VIAM_CARTO_POINTCLOUD_MAP_EMPTY;
    }
    VLOG(1) << "writing r->point_cloud_pcd. pointcloud_map.length(): "
            << pointcloud_map.length();
    r->point_cloud_pcd = to_bstring(pointcloud_map);
    VLOG(1) << "r->point_cloud_pcd.length(): " << blength(r->point_cloud_pcd);
};

void CartoFacade::GetInternalState(viam_carto_get_internal_state_response *r){};

void CartoFacade::Start() {
    started = true;
    StartSaveInternalState();
};

void CartoFacade::StartSaveInternalState() {
    if (config.map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_internal_state = std::make_unique<std::thread>(
        [&]() { this->SaveInternalStateOnInterval(); });
}

void CartoFacade::StopSaveInternalState() {
    started = false;
    if (config.map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_internal_state->join();
}

void CartoFacade::SaveInternalStateOnInterval() {
    auto check_for_shutdown_interval_usec =
        std::chrono::microseconds(checkForShutdownIntervalMicroseconds);
    while (started) {
        auto start = std::chrono::high_resolution_clock::now();
        // Sleep for config.map_rate_sec duration, but check frequently for
        // shutdown
        while (started) {
            std::chrono::duration<double, std::milli> time_elapsed_msec =
                std::chrono::high_resolution_clock::now() - start;
            if (time_elapsed_msec >= config.map_rate_sec) {
                VLOG(1) << "time_elapsed_msec >= config.map_rate_sec";
                break;
            }
            if (config.map_rate_sec - time_elapsed_msec >=
                check_for_shutdown_interval_usec) {
                VLOG(1) << "config.map_rate_sec - time_elapsed_msec >= "
                           "check_for_shutdown_interval_usec";
                std::this_thread::sleep_for(check_for_shutdown_interval_usec);
            } else {
                VLOG(1) << "else";
                std::this_thread::sleep_for(config.map_rate_sec -
                                            time_elapsed_msec);
                break;
            }
        }

        // Breakout without saving if the session has ended

        if (!started) {
            LOG(INFO) << "Saving final optimized internal state";
        }
        std::time_t t = std::time(nullptr);
        const std::string filename_with_timestamp =
            viam::carto_facade::io::MakeFilenameWithTimestamp(
                path_to_internal_state, t);

        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SaveMapToFile(true, filename_with_timestamp);
        if (!started) {
            LOG(INFO) << "Finished saving final optimized internal state";
            break;
        }
    }
}

void CartoFacade::Stop() { StopSaveInternalState(); };

void CartoFacade::AddSensorReading(const viam_carto_sensor_reading *sr) {
    if (biseq(config.component_reference, sr->sensor) == false) {
        VLOG(1) << "expected sensor: " << to_std_string(sr->sensor) << " to be "
                << config.component_reference;
        throw VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST;
    }
    std::string sensor_reading = to_std_string(sr->sensor_reading);
    if (sensor_reading.length() == 0) {
        throw VIAM_CARTO_SENSOR_READING_EMPTY;
    }

    int64_t sensor_reading_time_unix_micro = sr->sensor_reading_time_unix_micro;
    auto [success, measurement] =
        viam::carto_facade::util::carto_sensor_reading(
            sensor_reading, sensor_reading_time_unix_micro);
    if (!success) {
        throw VIAM_CARTO_SENSOR_READING_INVALID;
    }

    cartographer::transform::Rigid3d tmp_global_pose;
    bool update_latest_global_pose = false;

    if (map_builder_mutex.try_lock()) {
        map_builder.AddSensorData(measurement);
        auto local_poses = map_builder.GetLocalSlamResultPoses();
        VLOG(1) << "local_poses.size(): " << local_poses.size();
        // NOTE: The first time local_poses.size() goes positive will
        // be the second time that map_builder.AddSensorData() succeeds.
        // At that time the pose will still be zeroed out.
        // In the future we may want to allow callers of
        // CartoFacade::GetPosition to be able to distinguish between the states
        // of:
        // 1. Cartographer has not yet computed a map nor position as not enough
        // successful sensor readings have been provided yet.
        // 2. Cartographer has been initialized but still believes
        // that the robot is at the origin of the map.
        // In order to distinguish between these two states we would need to
        // return an error to the caller of viam_cartographer_get_position if
        // latest_global_pose has never been set within
        // CartoFacade::AddSensorReading.
        if (local_poses.size() > 0) {
            update_latest_global_pose = true;
            tmp_global_pose = map_builder.GetGlobalPose(local_poses.back());
            VLOG(1) << "updating tmp_global_pose tmp_global_pose: "
                    << tmp_global_pose;
        }
        map_builder_mutex.unlock();
        if (update_latest_global_pose) {
            VLOG(1) << "updating latest_global_pose";
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
            VLOG(1) << "latest_global_pose: " << latest_global_pose;
        }
        return;
    } else {
        throw VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK;
    }
};

viam::carto_facade::ActionMode determine_action_mode(
    std::string path_to_internal_state, std::chrono::seconds map_rate_sec) {
    // Check if there is an apriori map (internal state) in the
    // path_to_internal_state directory
    std::vector<std::string> internal_state_filenames =
        list_sorted_files_in_directory(path_to_internal_state);

    // Check if there is a *.pbstream internal state in the
    // path_to_internal_state directory
    for (auto filename : internal_state_filenames) {
        if (filename.find(".pbstream") != std::string::npos) {
            // There is an apriori map (internal state) present, so we're
            // running either in updating or localization mode.
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
        LOG(ERROR)
            << "set to localization mode (map_rate_sec = 0) but "
               "couldn't find apriori map (internal state) to localize on";
        throw VIAM_CARTO_SLAM_MODE_INVALID;
    }
    // This log line is needed by rdk integration tests.
    LOG(INFO) << "Running in mapping mode";
    return viam::carto_facade::ActionMode::MAPPING;
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
    if (google::IsGoogleLoggingInitialized()) {
        return VIAM_CARTO_LIB_ALREADY_INITIALIZED;
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
    if (ppVCL == nullptr) {
        return VIAM_CARTO_LIB_INVALID;
    }

    if (*ppVCL == nullptr) {
        return VIAM_CARTO_LIB_INVALID;
    }

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
    viam::carto_facade::CartoFacade *cf;

    try {
        cf = new viam::carto_facade::CartoFacade(pVCL, c, ac);
    } catch (int err) {
        free(vc);
        return err;
    } catch (std::exception &e) {
        free(vc);
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }

    try {
        cf->IOInit();
    } catch (int err) {
        delete cf;
        free(vc);
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        delete cf;
        free(vc);
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    vc->carto_obj = cf;

    // point to newly created viam_carto struct
    *ppVC = vc;
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_start(viam_carto *vc) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }
    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        cf->Start();
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_stop(viam_carto *vc) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        cf->Stop();
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_terminate(viam_carto **ppVC) {
    if (ppVC == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (*ppVC == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>((*ppVC)->carto_obj);
    delete cf;
    free((viam_carto *)*ppVC);
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_sensor_reading(viam_carto *vc,
                                         const viam_carto_sensor_reading *sr) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (sr == nullptr) {
        return VIAM_CARTO_SENSOR_READING_INVALID;
    }

    try {
        VLOG(1) << "sr->sensor_reading_time_unix_micro: "
                << sr->sensor_reading_time_unix_micro;
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        cf->AddSensorReading(sr);
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_sensor_reading_destroy(
    viam_carto_sensor_reading *sr) {
    if (sr == nullptr) {
        return VIAM_CARTO_SENSOR_READING_INVALID;
    }
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;
    // destroy sensor_reading
    rc = bdestroy(sr->sensor_reading);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    sr->sensor_reading = nullptr;

    // destroy sensor
    rc = bdestroy(sr->sensor);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    sr->sensor = nullptr;

    return return_code;
};

extern int viam_carto_get_position(viam_carto *vc,
                                   viam_carto_get_position_response *r) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (r == nullptr) {
        return VIAM_CARTO_GET_POSITION_RESPONSE_INVALID;
    }

    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>((vc)->carto_obj);
        cf->GetPosition(r);
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r) {
    if (r == nullptr) {
        return VIAM_CARTO_GET_POSITION_RESPONSE_INVALID;
    }
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
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (r == nullptr) {
        return VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVLALID;
    }
    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>((vc)->carto_obj);
        cf->GetPointCloudMap(r);
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }

    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r) {
    if (r == nullptr) {
        return VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVLALID;
    }
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;
    rc = bdestroy(r->point_cloud_pcd);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    r->point_cloud_pcd = nullptr;
    return return_code;
};

extern int viam_carto_get_internal_state(
    viam_carto *vc, viam_carto_get_internal_state_response *r) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r) {
    return VIAM_CARTO_SUCCESS;
};
