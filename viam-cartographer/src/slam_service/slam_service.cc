// This is an experimental integration of cartographer into RDK.
#include "slam_service.h"

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>
#include <iostream>
#include <string>

#include "../io/file_handler.h"
#include "../mapping/map_builder.h"
#include "../utils/slam_service_helpers.h"
#include "Eigen/Core"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

namespace {
// Number of bytes in a pixel
static const int bytesPerPixel = 4;
struct ColorARGB {
    unsigned char A;
    unsigned char R;
    unsigned char G;
    unsigned char B;
};

// Check if the green color channel is 0 to filter unobserved pixels which is
// set in DrawTexture. See:
// https://github.com/cartographer-project/cartographer/blob/ef00de231
// 7dcf7895b09f18cc4d87f8b533a019b/cartographer/io/submap_painter.cc#L206-L207
bool CheckIfEmptyPixel(ColorARGB pixel_color) { return (pixel_color.G == 0); }

// Convert the scale of a specified color channel from the given UCHAR
// range of 0 - 255 to an inverse probability range of 100 - 0
int CalculateProbabilityFromColorChannels(ColorARGB pixel_color) {
    unsigned char max_val = UCHAR_MAX;
    unsigned char min_val = 0;
    unsigned char max_prob = 100;
    unsigned char min_prob = 0;

    // Probability is currently determined solely by the red color channel
    unsigned char color_channel_val = pixel_color.R;
    unsigned char prob = (max_val - color_channel_val) * (max_prob - min_prob) /
                         (max_val - min_val);
    return prob;
}
}  // namespace

namespace viam {

std::atomic<bool> b_continue_session{true};

::grpc::Status SLAMServiceImpl::GetPosition(ServerContext *context,
                                            const GetPositionRequest *request,
                                            GetPositionResponse *response) {
    cartographer::transform::Rigid3d global_pose;
    {
        std::lock_guard<std::mutex> lk(viam_response_mutex);
        global_pose = latest_global_pose;
    }

    auto pos_vector = global_pose.translation();
    auto pos_quat = global_pose.rotation();

    // Set pose for our response
    Pose *myPose = response->mutable_pose();
    myPose->set_x(pos_vector.x());
    myPose->set_y(pos_vector.y());
    myPose->set_z(pos_vector.z());

    // Set extra for our response (currently stores quaternion)
    google::protobuf::Struct *q;
    google::protobuf::Struct *extra = response->mutable_extra();
    q = extra->mutable_fields()->operator[]("quat").mutable_struct_value();
    q->mutable_fields()->operator[]("real").set_number_value(pos_quat.w());
    q->mutable_fields()->operator[]("imag").set_number_value(pos_quat.x());
    q->mutable_fields()->operator[]("jmag").set_number_value(pos_quat.y());
    q->mutable_fields()->operator[]("kmag").set_number_value(pos_quat.z());

    // Set component_reference for our response
    response->set_component_reference(camera_name);

    return grpc::Status::OK;
}

::grpc::Status SLAMServiceImpl::GetPointCloudMap(
    ServerContext *context, const GetPointCloudMapRequest *request,
    ServerWriter<GetPointCloudMapResponse> *writer) {
    std::string pointcloud_map;
    // Write or grab the latest pointcloud map in form of a string
    try {
        std::shared_lock optimization_lock{optimization_shared_mutex,
                                           std::defer_lock};
        if (action_mode != ActionMode::LOCALIZING &&
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
            if (action_mode == ActionMode::LOCALIZING) {
                LOG(INFO)
                    << "In localization mode, using cached pointcloud map";
            } else {
                LOG(INFO)
                    << "Optimization is occuring, using cached pointcloud map";
            }
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            pointcloud_map = latest_pointcloud_map;
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Stopping Cartographer: error encoding pointcloud: "
                   << e.what();
        std::terminate();
    }

    if (pointcloud_map.empty()) {
        LOG(ERROR) << "map pointcloud does not have points yet";
        return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                            "map pointcloud does not have points yet");
    }

    std::string pcd_chunk;
    GetPointCloudMapResponse response;
    for (int start_index = 0; start_index < pointcloud_map.size();
         start_index += maximumGRPCByteChunkSize) {
        pcd_chunk =
            pointcloud_map.substr(start_index, maximumGRPCByteChunkSize);
        response.set_point_cloud_pcd_chunk(pcd_chunk);
        bool ok = writer->Write(response);
        if (!ok)
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "error while writing to stream: stream closed");
    }
    return grpc::Status::OK;
}

::grpc::Status SLAMServiceImpl::GetInternalState(
    ServerContext *context, const GetInternalStateRequest *request,
    ServerWriter<GetInternalStateResponse> *writer) {
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::string filename = path_to_map + "/" + "temp_internal_state_" +
                           boost::uuids::to_string(uuid) + ".pbstream";
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        bool ok = map_builder.SaveMapToFile(true, filename);
        if (!ok) {
            std::ostringstream oss;
            oss << "Failed to save the state as a pbstream.";
            return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
        }
    }

    std::string buf;
    // deferring reading the pbstream file in chunks until we run into issues
    // with loading the file into memory
    try {
        ConvertSavedMapToStream(filename, &buf);
    } catch (std::exception &e) {
        std::ostringstream oss;
        oss << "error during data serialization: " << e.what();
        return grpc::Status(grpc::StatusCode::UNAVAILABLE, oss.str());
    }

    std::string internal_state_chunk;
    GetInternalStateResponse response;
    for (int start_index = 0; start_index < buf.size();
         start_index += maximumGRPCByteChunkSize) {
        internal_state_chunk =
            buf.substr(start_index, maximumGRPCByteChunkSize);
        response.set_internal_state_chunk(internal_state_chunk);
        bool ok = writer->Write(response);
        if (!ok)
            return grpc::Status(grpc::StatusCode::UNAVAILABLE,
                                "error while writing to stream: stream closed");
    }

    return grpc::Status::OK;
}

void SLAMServiceImpl::ConvertSavedMapToStream(std::string filename,
                                              std::string *buffer) {
    std::stringstream error_forwarded;

    std::ifstream tempFile(filename);
    if (tempFile.bad()) {
        error_forwarded << "Failed to open " << filename
                        << " as ifstream object.";
        error_forwarded << TryFileClose(tempFile, filename);
        throw std::runtime_error(error_forwarded.str());
    }

    std::stringstream bufferStream;
    if (bufferStream << tempFile.rdbuf()) {
        *buffer = bufferStream.str();
    } else {
        error_forwarded << "Failed to get data from " << filename
                        << " to buffer stream.";
        error_forwarded << TryFileClose(tempFile, filename);
        throw std::runtime_error(error_forwarded.str());
    }

    error_forwarded << TryFileClose(tempFile, filename);

    if (std::remove(filename.c_str()) != 0) {
        error_forwarded << "Failed to delete " << filename;
        throw std::runtime_error(error_forwarded.str());
    }
}

std::string SLAMServiceImpl::TryFileClose(std::ifstream &tempFile,
                                          std::string filename) {
    tempFile.close();
    if (tempFile.bad()) {
        return (" Failed to close ifstream object " + filename);
    }
    return "";
}

void SLAMServiceImpl::BackupLatestMap() {
    std::string pointcloud_map_tmp;
    GetLatestSampledPointCloudMapString(pointcloud_map_tmp);
    std::lock_guard<std::mutex> lk(viam_response_mutex);
    latest_pointcloud_map = std::move(pointcloud_map_tmp);
}

// If using the LOCALIZING action mode, cache a copy of the map before
// beginning to process data. If cartographer fails to do this,
// terminate the program
void SLAMServiceImpl::CacheMapInLocalizationMode() {
    if (action_mode == ActionMode::LOCALIZING) {
        std::string pointcloud_map_tmp;
        try {
            GetLatestSampledPointCloudMapString(pointcloud_map_tmp);

        } catch (std::exception &e) {
            LOG(ERROR) << "Stopping Cartographer: error encoding localized "
                          "pointcloud map: "
                       << e.what();
            std::terminate();
        }

        if (pointcloud_map_tmp.empty()) {
            LOG(ERROR) << "Stopping Cartographer: error encoding localized "
                          "pointcloud map: no map points";
            std::terminate();
        }

        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_pointcloud_map = std::move(pointcloud_map_tmp);
        }
    }
}

ActionMode SLAMServiceImpl::GetActionMode() { return action_mode; }

void SLAMServiceImpl::SetActionMode() {
    action_mode = viam::utils::DetermineActionMode(path_to_map, map_rate_sec);
}

void SLAMServiceImpl::OverwriteMapBuilderParameters() {
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.OverwriteOptimizeEveryNNodes(optimize_every_n_nodes);
        map_builder.OverwriteNumRangeData(num_range_data);
        map_builder.OverwriteMissingDataRayLength(missing_data_ray_length);
        map_builder.OverwriteMaxRange(max_range);
        map_builder.OverwriteMinRange(min_range);
        if (action_mode == ActionMode::LOCALIZING) {
            map_builder.OverwriteMaxSubmapsToKeep(max_submaps_to_keep);
        }
        if (action_mode == ActionMode::UPDATING) {
            map_builder.OverwriteFreshSubmapsCount(fresh_submaps_count);
            map_builder.OverwriteMinCoveredArea(min_covered_area);
            map_builder.OverwriteMinAddedSubmapsCount(min_added_submaps_count);
        }
        map_builder.OverwriteOccupiedSpaceWeight(occupied_space_weight);
        map_builder.OverwriteTranslationWeight(translation_weight);
        map_builder.OverwriteRotationWeight(rotation_weight);
    }
}

void SLAMServiceImpl::SetUpMapBuilder() {
    if (action_mode == ActionMode::MAPPING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_mapping_basename);
    } else if (action_mode == ActionMode::LOCALIZING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_localization_basename);
    } else if (action_mode == ActionMode::UPDATING) {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SetUp(configuration_directory,
                          configuration_update_basename);
    } else {
        throw std::runtime_error("invalid action mode");
    }
    OverwriteMapBuilderParameters();
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    map_builder.BuildMapBuilder();
}

void SLAMServiceImpl::GetLatestSampledPointCloudMapString(
    std::string &pointcloud) {
    std::unique_ptr<cartographer::io::PaintSubmapSlicesResult> painted_slices =
        nullptr;
    try {
        painted_slices =
            std::make_unique<cartographer::io::PaintSubmapSlicesResult>(
                GetLatestPaintedMapSlices());
    } catch (std::exception &e) {
        if (e.what() == errorNoSubmaps) {
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
            if (CheckIfEmptyPixel(pixel_color)) {
                continue;
            }

            // Determine probability based on the color of the pixel and skip if
            // it is 0
            int prob = CalculateProbabilityFromColorChannels(pixel_color);
            if (prob == 0) {
                continue;
            }

            // Convert pixel location to pointcloud point in meters
            float x_pos = (pixel_x - origin_pixel_x) * resolutionMeters;
            // Y is inverted to match output from getPosition()
            float y_pos = -(pixel_y - origin_pixel_y) * resolutionMeters;
            float z_pos = 0;  // Z is 0 in 2D SLAM

            // Add point to buffer
            viam::utils::writeFloatToBufferInBytes(pcd_data, x_pos);
            viam::utils::writeFloatToBufferInBytes(pcd_data, y_pos);
            viam::utils::writeFloatToBufferInBytes(pcd_data, z_pos);
            viam::utils::writeIntToBufferInBytes(pcd_data, prob);

            num_points++;
        }
    }

    // Write our PCD file, which is written as a binary.
    pointcloud = viam::utils::pcdHeader(num_points, true);

    // Writes data buffer to the pointcloud string
    pointcloud += pcd_data;
    return;
}

cartographer::io::PaintSubmapSlicesResult
SLAMServiceImpl::GetLatestPaintedMapSlices() {
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
        throw std::runtime_error(errorNoSubmaps);
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

double SLAMServiceImpl::SetUpSLAM() {
    // Setting the action mode has to happen before setting up the
    // map builder.
    SetActionMode();
    // Set up and build the MapBuilder
    SetUpMapBuilder();

    double data_start_time = 0;
    if (action_mode == ActionMode::UPDATING ||
        action_mode == ActionMode::LOCALIZING) {
        // Check if there is an apriori map in the path_to_map directory
        std::string latest_map_filename =
            viam::utils::GetLatestMapFilename(path_to_map);
        // load_frozen_trajectory has to be true for LOCALIZING action mode,
        // and false for UPDATING action mode.
        bool load_frozen_trajectory = (action_mode == ActionMode::LOCALIZING);
        if (optimize_on_start) {
            BackupLatestMap();
            std::unique_lock optimization_lock{optimization_shared_mutex,
                                               std::defer_lock};
            optimization_lock.lock();
            // Load apriori map
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.LoadMapFromFile(
                latest_map_filename, load_frozen_trajectory, optimize_on_start);
        } else {
            // Load apriori map
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            map_builder.LoadMapFromFile(
                latest_map_filename, load_frozen_trajectory, optimize_on_start);
        }
        data_start_time =
            viam::io::ReadTimeFromTimestamp(latest_map_filename.substr(
                latest_map_filename.find(viam::io::filename_prefix) +
                    viam::io::filename_prefix.length(),
                latest_map_filename.find(".pbstream")));

        CacheMapInLocalizationMode();
    }
    return data_start_time;
}

void SLAMServiceImpl::RunSLAM() {
    LOG(INFO) << "Setting up cartographer";
    double data_start_time = SetUpSLAM();
    LOG(INFO) << "Starting to run cartographer";
    ProcessDataAndStartSavingMaps(data_start_time);
    LOG(INFO) << "Done running cartographer";
}

std::string SLAMServiceImpl::GetNextDataFileOffline() {
    if (!b_continue_session) {
        return "";
    }
    if (file_list_offline.size() == 0) {
        file_list_offline = viam::io::ListSortedFilesInDirectory(path_to_data);
    }
    // We're setting the minimum required files to be two for the following
    // reasons:
    // 1. Cartographer needs at least two PCD files to work properly.
    // 2. A .DS_Store file is frequently added to the data directory when
    // a user opens the directory on osx.
    // Expecting a minimum of 3 files solves both problems without having to
    // loop over and count the number of actual data files in the data
    // directory.
    if (file_list_offline.size() <= 2) {
        throw std::runtime_error("not enough data in data directory");
    }
    if (current_file_offline == file_list_offline.size()) {
        // This log line is needed by rdk integration tests.
        LOG(INFO) << "Finished processing offline data";
        return "";
    }
    const auto to_return = file_list_offline[current_file_offline];
    current_file_offline++;
    return to_return;
}

std::string SLAMServiceImpl::GetNextDataFileOnline() {
    while (b_continue_session) {
        const auto file_list_online =
            viam::io::ListSortedFilesInDirectory(path_to_data);
        if (delete_processed_data && first_processed_file_index >= 0) {
            for (int i = first_processed_file_index;
                 i < int(file_list_online.size()) - data_buffer_size; i++) {
                viam::io::RemoveFile(file_list_online.at(i));
            }
        }
        if (file_list_online.size() > 1) {
            // Get the second-most-recent file, since the most-recent file may
            // still be being written.
            const auto to_return =
                file_list_online[file_list_online.size() - 2];
            if (to_return.compare(current_file_online) != 0) {
                current_file_online = to_return;
                return to_return;
            }
        }
        VLOG(1) << "No new files found";
        std::this_thread::sleep_for(data_rate_ms);
    }
    return "";
}

std::string SLAMServiceImpl::GetNextDataFile() {
    if (use_live_data) {
        return GetNextDataFileOnline();
    }
    return GetNextDataFileOffline();
}

void SLAMServiceImpl::StartSaveMap() {
    if (map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_map_with_timestamp =
        new std::thread([&]() { this->SaveMapWithTimestamp(); });
}

void SLAMServiceImpl::StopSaveMap() {
    if (map_rate_sec == std::chrono::seconds(0)) {
        return;
    }
    thread_save_map_with_timestamp->join();
}

void SLAMServiceImpl::SaveMapWithTimestamp() {
    auto check_for_shutdown_interval_usec =
        std::chrono::microseconds(checkForShutdownIntervalMicroseconds);
    while (b_continue_session) {
        auto start = std::chrono::high_resolution_clock::now();
        // Sleep for map_rate_sec duration, but check frequently for
        // shutdown
        while (b_continue_session) {
            std::chrono::duration<double, std::milli> time_elapsed_msec =
                std::chrono::high_resolution_clock::now() - start;
            if ((time_elapsed_msec >= map_rate_sec) ||
                (!use_live_data && finished_processing_offline)) {
                break;
            }
            if (map_rate_sec - time_elapsed_msec >=
                check_for_shutdown_interval_usec) {
                std::this_thread::sleep_for(check_for_shutdown_interval_usec);
            } else {
                std::this_thread::sleep_for(map_rate_sec - time_elapsed_msec);
                break;
            }
        }

        // Breakout without saving if the session has ended
        if (!b_continue_session) {
            break;
        }

        std::time_t t = std::time(nullptr);
        const std::string filename_with_timestamp =
            viam::io::MakeFilenameWithTimestamp(path_to_map, t);

        if (!use_live_data && finished_processing_offline) {
            {
                std::lock_guard<std::mutex> lk(map_builder_mutex);
                map_builder.SaveMapToFile(true, filename_with_timestamp);
            }
            LOG(INFO) << "Finished saving final optimized map";
            return;
        }

        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.SaveMapToFile(true, filename_with_timestamp);
    }
}

void SLAMServiceImpl::ProcessDataAndStartSavingMaps(double data_start_time) {
    // Prepare the trajectory builder and grab the active trajectory_id
    cartographer::mapping::TrajectoryBuilderInterface *trajectory_builder;
    int trajectory_id;
    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        // Set TrajectoryBuilder
        trajectory_id = map_builder.SetTrajectoryBuilder(&trajectory_builder,
                                                         {kRangeSensorId});
        VLOG(1) << "Using trajectory ID: " << trajectory_id;
    }

    LOG(INFO) << "Beginning to add data...";

    bool set_start_time = false;
    auto file = GetNextDataFile();

    // Define tmp_global_pose here so it always has the previous pose
    cartographer::transform::Rigid3d tmp_global_pose =
        cartographer::transform::Rigid3d();
    while (file != "") {
        // Ignore files that are not *.pcd files
        if (file.find(".pcd") == std::string::npos) {
            file = GetNextDataFile();
            continue;
        }
        if (!set_start_time) {
            // Go past files that are not supposed to be included in this run
            double file_time = viam::io::ReadTimeFromTimestamp(
                file.substr(file.find(viam::io::filename_prefix) +
                                viam::io::filename_prefix.length(),
                            file.find(".pcd")));
            if (file_time < data_start_time) {
                file = GetNextDataFile();
                continue;
            }
            // Get index of the first file we're reading in
            const auto files =
                viam::io::ListSortedFilesInDirectory(path_to_data);
            auto file_index = std::find(begin(files), end(files), file);
            if (file_index == std::end(files)) {
                throw std::runtime_error(
                    "the file should be in the list of files: " + file);
            }
            first_processed_file_index =
                std::distance(files.begin(), file_index);

            // Set the start time if it has not yet been set and
            // start saving maps
            {
                std::lock_guard<std::mutex> lk(map_builder_mutex);
                map_builder.SetStartTime(file);
                set_start_time = true;
            }
            LOG(INFO) << "Starting to save maps...";
            StartSaveMap();
        }
        // Add data to the map_builder to add to the map
        {
            std::lock_guard<std::mutex> lk(map_builder_mutex);
            auto measurement = map_builder.GetDataFromFile(file);
            if (measurement.ranges.size() > 0) {
                trajectory_builder->AddSensorData(kRangeSensorId.id,
                                                  measurement);
                auto local_poses = map_builder.GetLocalSlamResultPoses();
                if (local_poses.size() > 0) {
                    tmp_global_pose = map_builder.GetGlobalPose(
                        trajectory_id, local_poses.back());
                }
            }
        }
        // Save a copy of the global pose
        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }

        // This log line is needed by rdk integration tests.
        VLOG(1) << "Passed sensor data to SLAM " << file;

        file = GetNextDataFile();
    }

    if (!set_start_time) {
        throw std::runtime_error("did not find valid data for the given setup");
    }

    {
        std::lock_guard<std::mutex> lk(map_builder_mutex);
        map_builder.map_builder_->FinishTrajectory(trajectory_id);
    }
    if (!use_live_data) {
        // We still want to optimize the map in localization mode, but we do not
        // need to update the backup of the map
        if (action_mode != ActionMode::LOCALIZING) BackupLatestMap();
        {
            std::unique_lock<std::shared_mutex> optimization_lock(
                optimization_shared_mutex, std::defer_lock);
            optimization_lock.lock();

            std::lock_guard<std::mutex> lk(map_builder_mutex);
            LOG(INFO) << "Starting to optimize final map. This can take a "
                         "little while...";
            map_builder.map_builder_->pose_graph()->RunFinalOptimization();

            auto local_poses = map_builder.GetLocalSlamResultPoses();
            if (local_poses.size() > 0) {
                tmp_global_pose = map_builder.GetGlobalPose(trajectory_id,
                                                            local_poses.back());
            }
        }

        {
            std::lock_guard<std::mutex> lk(viam_response_mutex);
            latest_global_pose = tmp_global_pose;
        }

        finished_processing_offline = true;
        // This log line is needed by rdk integration tests.
        VLOG(1) << "Finished optimizing final map";

        while (viam::b_continue_session) {
            VLOG(1) << "Standing by to continue serving requests";
            std::this_thread::sleep_for(std::chrono::microseconds(
                viam::checkForShutdownIntervalMicroseconds));
        }
    }
    StopSaveMap();
    LOG(INFO) << "Stopped saving maps";
    return;
}

int SLAMServiceImpl::GetOptimizeEveryNNodesFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetOptimizeEveryNNodes();
}

int SLAMServiceImpl::GetNumRangeDataFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetNumRangeData();
}

float SLAMServiceImpl::GetMissingDataRayLengthFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMissingDataRayLength();
}

float SLAMServiceImpl::GetMaxRangeFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMaxRange();
}

float SLAMServiceImpl::GetMinRangeFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMinRange();
}

int SLAMServiceImpl::GetMaxSubmapsToKeepFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMaxSubmapsToKeep();
}

int SLAMServiceImpl::GetFreshSubmapsCountFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetFreshSubmapsCount();
}

double SLAMServiceImpl::GetMinCoveredAreaFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMinCoveredArea();
}

int SLAMServiceImpl::GetMinAddedSubmapsCountFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetMinAddedSubmapsCount();
}

double SLAMServiceImpl::GetOccupiedSpaceWeightFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetOccupiedSpaceWeight();
}

double SLAMServiceImpl::GetTranslationWeightFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetTranslationWeight();
}

double SLAMServiceImpl::GetRotationWeightFromMapBuilder() {
    std::lock_guard<std::mutex> lk(map_builder_mutex);
    return map_builder.GetRotationWeight();
}

}  // namespace viam
