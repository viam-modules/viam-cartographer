// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>

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

std::ostream &operator<<(std::ostream &os,
                         const viam::carto_facade::SlamMode &slam_mode) {
    std::string slam_mode_str;
    if (slam_mode == viam::carto_facade::SlamMode::MAPPING) {
        slam_mode_str = "mapping";
    } else if (slam_mode == viam::carto_facade::SlamMode::LOCALIZING) {
        slam_mode_str = "localizing";
    } else if (slam_mode == viam::carto_facade::SlamMode::UPDATING) {
        slam_mode_str = "updating";
    } else {
        throw std::runtime_error("invalid viam::carto_facade::SlamMode value");
    }
    os << slam_mode_str;
    return os;
}

std::ostream &operator<<(std::ostream &os, const CartoFacadeState &state) {
    std::string state_str;
    if (state == CartoFacadeState::INITIALIZED) {
        state_str = "initialized";
    } else if (state == CartoFacadeState::IO_INITIALIZED) {
        state_str = "io_initialized";
    } else if (state == CartoFacadeState::STARTED) {
        state_str = "started";
    } else {
        throw std::runtime_error("invalid CartoFacadeState value");
    }
    os << state_str;
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
    c.camera = to_std_string(vcc.camera);
    c.movement_sensor = to_std_string(vcc.movement_sensor);
    c.data_dir = to_std_string(vcc.data_dir);
    c.map_rate_sec = std::chrono::seconds(vcc.map_rate_sec);
    c.cloud_story_enabled = vcc.cloud_story_enabled;
    c.enable_mapping = vcc.enable_mapping;
    c.existing_map = to_std_string(vcc.existing_map);
    c.lidar_config = vcc.lidar_config;

    if (!c.cloud_story_enabled) {
        if (c.data_dir.size() == 0) {
            throw VIAM_CARTO_DATA_DIR_NOT_PROVIDED;
        }
        if (vcc.map_rate_sec < 0) {
            throw VIAM_CARTO_MAP_RATE_SEC_INVALID;
        }
    }

    if (c.camera.empty()) {
        throw VIAM_CARTO_COMPONENT_REFERENCE_INVALID;
    }
    validate_lidar_config(c.lidar_config);
    c.component_reference = bstrcpy(vcc.camera);

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

const std::string slam_mode_lua_config_filename(
    viam::carto_facade::SlamMode sm) {
    switch (sm) {
        case viam::carto_facade::SlamMode::MAPPING:
            return configuration_mapping_basename;
            break;
        case viam::carto_facade::SlamMode::LOCALIZING:
            return configuration_localization_basename;
            break;
        case viam::carto_facade::SlamMode::UPDATING:
            return configuration_update_basename;
            break;
        default:
            LOG(ERROR) << "slam_mode_lua_config_filename: slam mode is invalid";
            throw VIAM_CARTO_SLAM_MODE_INVALID;
    }
};

CartoFacade::CartoFacade(viam_carto_lib *pVCL, const viam_carto_config c,
                         const viam_carto_algo_config ac) {
    lib = pVCL;
    config = from_viam_carto_config(c);
    algo_config = ac;
    path_to_internal_state = config.data_dir + "/internal_state";
    path_to_internal_state_file = config.existing_map;
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
    if (state != CartoFacadeState::INITIALIZED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::INITIALIZED;
        throw VIAM_CARTO_NOT_IN_INITIALIZED_STATE;
    }
    if (config.cloud_story_enabled == true) {
        slam_mode = determine_slam_mode_cloud_story_enabled(
            path_to_internal_state_file, config.enable_mapping);
    } else {
        // Detect if data_dir has deprecated format
        if (fs::is_directory(config.data_dir + "/data")) {
            LOG(ERROR)
                << "data directory " << config.data_dir
                << " is invalid as it contains deprecated format i.e. /data "
                   "subdirectory";
            throw VIAM_CARTO_DATA_DIR_INVALID_DEPRECATED_STRUCTURE;
        }
        // Setup file system for saving internal state
        setup_filesystem(config.data_dir, path_to_internal_state);
        slam_mode =
            determine_slam_mode(path_to_internal_state, config.map_rate_sec);
    }

    VLOG(1) << "slam mode: " << slam_mode;
    // TODO: Make this API user configurable
    auto cd = find_lua_files();
    if (cd.empty()) {
        throw VIAM_CARTO_LUA_CONFIG_NOT_FOUND;
    }
    configuration_directory = cd;
    // Detect slam mode
    auto config_basename = slam_mode_lua_config_filename(slam_mode);
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
        map_builder.OverwriteUseIMUData(algo_config.use_imu_data);
        if (slam_mode == viam::carto_facade::SlamMode::LOCALIZING) {
            map_builder.OverwriteMaxSubmapsToKeep(
                algo_config.max_submaps_to_keep);
        }
        if (slam_mode == viam::carto_facade::SlamMode::UPDATING) {
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

    // TODO: google cartographer will terminate the program if
    // the internal state is invalid
    // see https://viam.atlassian.net/browse/RSDK-3553
    if (slam_mode == viam::carto_facade::SlamMode::UPDATING ||
        slam_mode == viam::carto_facade::SlamMode::LOCALIZING) {
        // Check if there is an apriori map (internal state) in the
        // path_to_internal_state directory or existing_map path
        std::string latest_internal_state_filename;
        if (config.cloud_story_enabled) {
            latest_internal_state_filename = config.existing_map;
        } else {
            latest_internal_state_filename =
                get_latest_internal_state_filename(path_to_internal_state);
        }

        VLOG(1) << "latest_internal_state_filename: "
                << latest_internal_state_filename;
        // load_frozen_trajectory has to be true for LOCALIZING slam mode,
        // and false for UPDATING slam mode.
        bool load_frozen_trajectory =
            (slam_mode == viam::carto_facade::SlamMode::LOCALIZING);
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
        map_builder.StartTrajectoryBuilder(algo_config.use_imu_data);
    }
    state = CartoFacadeState::IO_INITIALIZED;
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

// If using the LOCALIZING slam mode, cache a copy of the map before
// beginning to process data. If cartographer fails to do this,
// terminate the program
void CartoFacade::CacheMapInLocalizationMode() {
    VLOG(1) << "CacheMapInLocalizationMode()";
    if (slam_mode == viam::carto_facade::SlamMode::LOCALIZING) {
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

void CartoFacade::RunFinalOptimization() {
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.map_builder_->pose_graph()->RunFinalOptimization();
    }
}

void CartoFacade::GetPosition(viam_carto_get_position_response *r) {
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    cartographer::transform::Rigid3d global_pose;
    {
        std::lock_guard<std::mutex> lk(viam_response_mutex);
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
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    std::string pointcloud_map;
    // Write or grab the latest pointcloud map in form of a string
    std::shared_lock optimization_lock{optimization_shared_mutex,
                                       std::defer_lock};
    if (slam_mode != viam::carto_facade::SlamMode::LOCALIZING &&
        optimization_lock.try_lock()) {
        // We are able to lock the optimization_shared_mutex, which means
        // that the optimization is not ongoing and we can grab the newest
        // map
        GetLatestSampledPointCloudMapString(pointcloud_map);
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        latest_pointcloud_map = pointcloud_map;
    } else {
        // Either we are in localization mode or we couldn't lock the mutex
        // which means the optimization process locked it and we need to use
        // the backed up latest map
        if (slam_mode == viam::carto_facade::SlamMode::LOCALIZING) {
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
    r->point_cloud_pcd = to_bstring(pointcloud_map);
};

// TODO: This function is unnecessarily prone to IO errors
// due to going through the file system in order to read
// the internal state.
// This is the ticket to remove that failure mode:
// https://viam.atlassian.net/browse/RSDK-3878
void CartoFacade::GetInternalState(viam_carto_get_internal_state_response *r) {
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    boost::uuids::uuid uuid = boost::uuids::random_generator()();

    std::string filename;
    if (config.cloud_story_enabled) {
        filename = "temp_internal_state_" + boost::uuids::to_string(uuid) +
                   ".pbstream";
    } else {
        filename = path_to_internal_state + "/" + "temp_internal_state_" +
                   boost::uuids::to_string(uuid) + ".pbstream";
    }

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        bool ok = map_builder.SaveMapToFile(true, filename);
        if (!ok) {
            LOG(ERROR) << "Failed to save the internal state as a pbstream.";
            throw VIAM_CARTO_GET_INTERNAL_STATE_FILE_WRITE_IO_ERROR;
        }
    }

    std::string internal_state;
    try {
        viam::carto_facade::util::read_and_delete_file(filename,
                                                       &internal_state);
    } catch (std::exception &e) {
        LOG(ERROR) << "Failed to read and/or delete internal state file: "
                   << e.what();
        throw VIAM_CARTO_GET_INTERNAL_STATE_FILE_READ_IO_ERROR;
    }
    r->internal_state = to_bstring(internal_state);
};

void CartoFacade::Start() {
    if (state != CartoFacadeState::IO_INITIALIZED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::IO_INITIALIZED;
        throw VIAM_CARTO_NOT_IN_IO_INITIALIZED_STATE;
    }
    state = CartoFacadeState::STARTED;
    if (!config.cloud_story_enabled) {
        StartSaveInternalState();
    }
};

void CartoFacade::StartSaveInternalState() {
    if (config.map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_internal_state = std::make_unique<std::thread>(
        [&]() { this->SaveInternalStateOnInterval(); });
}

void CartoFacade::StopSaveInternalState() {
    if (config.map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_internal_state->join();
}

void CartoFacade::SaveInternalStateOnInterval() {
    auto check_for_shutdown_interval_usec =
        std::chrono::microseconds(checkForShutdownIntervalMicroseconds);
    while (state == CartoFacadeState::STARTED) {
        auto start = std::chrono::high_resolution_clock::now();
        // Sleep for config.map_rate_sec duration, but check frequently for
        // shutdown
        while (state == CartoFacadeState::STARTED) {
            std::chrono::duration<double, std::milli> time_elapsed_msec =
                std::chrono::high_resolution_clock::now() - start;
            if (time_elapsed_msec >= config.map_rate_sec) {
                break;
            }
            if (config.map_rate_sec - time_elapsed_msec >=
                check_for_shutdown_interval_usec) {
                std::this_thread::sleep_for(check_for_shutdown_interval_usec);
            } else {
                std::this_thread::sleep_for(config.map_rate_sec -
                                            time_elapsed_msec);
                break;
            }
        }

        // Breakout without saving if the session has ended

        if (state != CartoFacadeState::STARTED) {
            LOG(INFO) << "Saving final optimized internal state";
        }
        std::time_t t = std::time(nullptr);
        const std::string filename_with_timestamp =
            viam::carto_facade::io::MakeFilenameWithTimestamp(
                path_to_internal_state, t);

        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SaveMapToFile(true, filename_with_timestamp);
        if (state != CartoFacadeState::STARTED) {
            LOG(INFO) << "Finished saving final optimized internal state";
            break;
        }
    }
}

void CartoFacade::Stop() {
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state << " expected "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    state = CartoFacadeState::IO_INITIALIZED;
    if (!config.cloud_story_enabled) {
        StopSaveInternalState();
    }
};

void CartoFacade::AddLidarReading(const viam_carto_lidar_reading *sr) {
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state
                   << " expected it to be in state: "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    if (biseq(config.component_reference, sr->lidar) == false) {
        VLOG(1) << "expected sensor: " << to_std_string(sr->lidar) << " to be "
                << config.component_reference;
        throw VIAM_CARTO_UNKNOWN_SENSOR_NAME;
    }
    std::string lidar_reading = to_std_string(sr->lidar_reading);
    if (lidar_reading.length() == 0) {
        throw VIAM_CARTO_LIDAR_READING_EMPTY;
    }

    int64_t lidar_reading_time_unix_milli = sr->lidar_reading_time_unix_milli;
    auto [success, measurement] = viam::carto_facade::util::carto_lidar_reading(
        lidar_reading, lidar_reading_time_unix_milli);
    if (!success) {
        throw VIAM_CARTO_LIDAR_READING_INVALID;
    }

    cartographer::transform::Rigid3d tmp_global_pose;

    if (map_builder_mutex.try_lock()) {
        VLOG(1) << "AddSensorData timestamp: " << measurement.time
                << " Sensor type: Lidar "
                << " measurement.ranges.size(): " << measurement.ranges.size();
        map_builder.AddSensorData(kRangeSensorId.id, measurement);
        tmp_global_pose = map_builder.GetGlobalPose();
        map_builder_mutex.unlock();
        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }
        return;
    } else {
        throw VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK;
    }
};

void CartoFacade::AddIMUReading(const viam_carto_imu_reading *sr) {
    if (state != CartoFacadeState::STARTED) {
        LOG(ERROR) << "carto facade is in state: " << state
                   << " expected it to be in state: "
                   << CartoFacadeState::STARTED;
        throw VIAM_CARTO_NOT_IN_STARTED_STATE;
    }
    if (biseq(to_bstring(config.movement_sensor), sr->imu) == false) {
        VLOG(1) << "expected sensor: " << to_std_string(sr->imu) << " to be "
                << config.movement_sensor;
        throw VIAM_CARTO_UNKNOWN_SENSOR_NAME;
    }

    int64_t imu_reading_time_unix_milli = sr->imu_reading_time_unix_milli;

    cartographer::sensor::ImuData measurement;
    measurement.time =
        cartographer::common::FromUniversal(0) +
        cartographer::common::FromMilliseconds(imu_reading_time_unix_milli);
    measurement.linear_acceleration =
        Eigen::Vector3d(sr->lin_acc_x, sr->lin_acc_y, sr->lin_acc_z);
    measurement.angular_velocity =
        Eigen::Vector3d(sr->ang_vel_x, sr->ang_vel_y, sr->ang_vel_z);

    cartographer::transform::Rigid3d tmp_global_pose;

    if (map_builder_mutex.try_lock()) {
        VLOG(1) << "AddSensorData timestamp: " << measurement.time
                << " Sensor type: IMU ";
        map_builder.AddSensorData(kIMUSensorId.id, measurement);
        VLOG(1) << "Data added is: " << measurement.linear_acceleration
                << " and " << measurement.angular_velocity;
        LOG(INFO) << "Added IMU data to Cartographer";
        tmp_global_pose = map_builder.GetGlobalPose();
        map_builder_mutex.unlock();
        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }
        return;
    } else {
        throw VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK;
    }
};

viam::carto_facade::SlamMode determine_slam_mode(
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
                return viam::carto_facade::SlamMode::LOCALIZING;
            }
            // This log line is needed by rdk integration tests.
            LOG(INFO) << "Running in updating mode";
            return viam::carto_facade::SlamMode::UPDATING;
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
    return viam::carto_facade::SlamMode::MAPPING;
}

viam::carto_facade::SlamMode determine_slam_mode_cloud_story_enabled(
    std::string path_to_internal_state_file, bool enable_mapping) {
    // Check if an existing map has been provided
    if (path_to_internal_state_file.size() != 0) {
        // There is an apriori map (internal state) present, so we're
        // running either in updating or localization mode.
        if (!enable_mapping) {
            // This log line is needed by rdk integration tests.
            LOG(INFO) << "Running in localization only mode";
            return viam::carto_facade::SlamMode::LOCALIZING;
        }
        // This log line is needed by rdk integration tests.
        LOG(INFO) << "Running in updating mode";
        return viam::carto_facade::SlamMode::UPDATING;
    }
    if (!enable_mapping) {
        LOG(ERROR)
            << "set to localization mode (enable_mapping = false) but "
               "couldn't find apriori map (internal state) to localize on";
        throw VIAM_CARTO_SLAM_MODE_INVALID;
    }
    // This log line is needed by rdk integration tests.
    LOG(INFO) << "Running in mapping mode";
    return viam::carto_facade::SlamMode::MAPPING;
}

int slam_mode_to_vc_slam_mode(viam::carto_facade::SlamMode sm) {
    if (sm == viam::carto_facade::SlamMode::MAPPING) {
        return VIAM_CARTO_SLAM_MODE_MAPPING;
    } else if (sm == viam::carto_facade::SlamMode::LOCALIZING) {
        return VIAM_CARTO_SLAM_MODE_LOCALIZING;
    } else if (sm == viam::carto_facade::SlamMode::UPDATING) {
        return VIAM_CARTO_SLAM_MODE_UPDATING;
    } else {
        throw std::runtime_error("invalid viam::carto_facade::SlamMode value");
    }
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
    // check that IMU is correctly set up
    if ((ac.use_imu_data == true &&
         biseqcstr(c.movement_sensor, ("")) == true) ||
        (ac.use_imu_data == false &&
         biseqcstr(c.movement_sensor, ("")) == false)) {
        return VIAM_CARTO_IMU_ENABLED_INVALID;
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

    int slam_mode = VIAM_CARTO_SLAM_MODE_UNKNOWN;
    try {
        cf->IOInit();
        slam_mode =
            viam::carto_facade::slam_mode_to_vc_slam_mode(cf->slam_mode);
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
    vc->slam_mode = slam_mode;

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
    if (cf->state != viam::carto_facade::CartoFacadeState::INITIALIZED &&
        cf->state != viam::carto_facade::CartoFacadeState::IO_INITIALIZED) {
        LOG(ERROR) << "carto facade is in state: " << cf->state << " expected "
                   << viam::carto_facade::CartoFacadeState::INITIALIZED
                   << " or "
                   << viam::carto_facade::CartoFacadeState::IO_INITIALIZED;
        return VIAM_CARTO_NOT_IN_TERMINATABLE_STATE;
    }
    delete cf;
    free((viam_carto *)*ppVC);
    *ppVC = nullptr;
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_lidar_reading(viam_carto *vc,
                                        const viam_carto_lidar_reading *sr) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (sr == nullptr) {
        return VIAM_CARTO_LIDAR_READING_INVALID;
    }

    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        cf->AddLidarReading(sr);
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_lidar_reading_destroy(viam_carto_lidar_reading *sr) {
    if (sr == nullptr) {
        return VIAM_CARTO_LIDAR_READING_INVALID;
    }
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;
    // destroy lidar_reading
    rc = bdestroy(sr->lidar_reading);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    sr->lidar_reading = nullptr;

    // destroy sensor
    rc = bdestroy(sr->lidar);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    sr->lidar = nullptr;

    return return_code;
};

extern int viam_carto_add_imu_reading(viam_carto *vc,
                                      const viam_carto_imu_reading *sr) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (sr == nullptr) {
        return VIAM_CARTO_IMU_READING_INVALID;
    }

    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>(vc->carto_obj);
        cf->AddIMUReading(sr);
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_imu_reading_destroy(viam_carto_imu_reading *sr) {
    if (sr == nullptr) {
        return VIAM_CARTO_IMU_READING_INVALID;
    }
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;

    // destroy sensor
    rc = bdestroy(sr->imu);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    sr->imu = nullptr;

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
        return VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVALID;
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
        return VIAM_CARTO_GET_POINT_CLOUD_MAP_RESPONSE_INVALID;
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
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    if (r == nullptr) {
        return VIAM_CARTO_GET_INTERNAL_STATE_RESPONSE_INVALID;
    }
    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>((vc)->carto_obj);
        cf->GetInternalState(r);
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }

    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r) {
    if (r == nullptr) {
        return VIAM_CARTO_GET_INTERNAL_STATE_RESPONSE_INVALID;
    }
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;
    rc = bdestroy(r->internal_state);
    if (rc != BSTR_OK) {
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    r->internal_state = nullptr;
    return return_code;
};

extern int viam_carto_run_final_optimization(viam_carto *vc) {
    if (vc == nullptr) {
        return VIAM_CARTO_VC_INVALID;
    }

    try {
        viam::carto_facade::CartoFacade *cf =
            static_cast<viam::carto_facade::CartoFacade *>((vc)->carto_obj);
        cf->RunFinalOptimization();
    } catch (int err) {
        return err;
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
        return VIAM_CARTO_UNKNOWN_ERROR;
    }

    return VIAM_CARTO_SUCCESS;
};
