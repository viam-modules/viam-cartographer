// This is an experimental integration of cartographer into RDK.
#include "io.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/console/time.h>  // pcl::console::TicToc
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>  // pcl::PCDReader
#include <pcl/point_types.h>
#include <stdio.h>

#include <boost/filesystem.hpp>
#include <exception>
#include <fstream>  // std::ifstream
#include <iomanip>
#include <iostream>
#include <sstream>  // std::istringstream
#include <utility>

#include "glog/logging.h"

namespace viam {
namespace carto_facade {
namespace io {

namespace fs = boost::filesystem;

const std::string MakeFilenameWithTimestamp(std::string path_to_dir,
                                            std::time_t t) {
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), time_format.c_str(),
                  std::gmtime(&t));
    return path_to_dir + "/" + "map_data_" + timestamp + ".pbstream";
}

int readPCD(std::string pcd, pcl::PCLPointCloud2 &blob) {
    pcl::PCDReader p;

    int pcd_version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::console::TicToc tt;
    tt.tic();

    int data_type;
    unsigned int data_idx;

    std::istringstream pcd_stream(pcd);
    int res = p.readHeader(pcd_stream, blob, origin, orientation, pcd_version,

                           data_type, data_idx);
    if (res != 0) {
        LOG(ERROR) << "Failed to parse header";
    } else {
        switch (data_type) {
            // ascii
            case 0:
                pcd_stream.seekg(data_idx);
                res = p.readBodyASCII(pcd_stream, blob, pcd_version);
                break;
            // binary
            case 1: {
                // This block exists b/c otherwise
                // map is counterintuitively visible
                // to the rest of the case
                // statement branches
                std::size_t expected_size = data_idx + blob.data.size();
                if (expected_size > pcd.length()) {
                    LOG(ERROR) << "Corrupted PCD file. The file is smaller "
                                  "than expected!";
                    return -1;
                }
                const unsigned char *map =
                    reinterpret_cast<const unsigned char *>(pcd.c_str());
                res = p.readBodyBinary(map, blob, pcd_version, false, data_idx);
            } break;
            // binary compressed
            case 2:

                LOG(ERROR) << "compressed PCDs are not currently supported";
                res = -1;
                break;
            default:
                LOG(ERROR) << "PCD classified as an unsupported data type: "
                           << data_type;
                res = -1;
        }
    }

    double total_time = tt.toc();
    VLOG(1) << "[viam::carto_facade::io::readPCD] Loaded as a "
            << (blob.is_dense ? "dense" : "non-dense") << "blob in "
            << total_time << "ms with " << blob.width * blob.height
            << "points. Available dimensions: "
            << pcl::getFieldsList(blob).c_str();
    return res;
}

std::tuple<bool, cartographer::sensor::TimedPointCloudData> ToSensorData(
    std::string sensor_reading, long long sensor_reading_time_unix_micro) {
    cartographer::sensor::TimedPointCloudData point_cloud;
    cartographer::sensor::TimedPointCloud ranges;

    pcl::PCLPointCloud2 blob;

    try {
        int err = readPCD(sensor_reading, blob);
        if (err) {
            return {false, point_cloud};
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "exception thrown during readPCD: " << e.what();
        return {false, point_cloud};
    }
    LOG(INFO) << "readPCD succeeded";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(blob, *cloud);

    VLOG(1) << "Loaded " << cloud->width * cloud->height << " data points";

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cartographer::sensor::TimedRangefinderPoint timed_rangefinder_point;
        timed_rangefinder_point.position = Eigen::Vector3f(
            cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        // TODO: Why are we doing this?
        timed_rangefinder_point.time = 0 - i * 0.0001;

        ranges.push_back(timed_rangefinder_point);
    }

    // research how to do this properly
    point_cloud.time =
        cartographer::common::FromUniversal(sensor_reading_time_unix_micro);
    point_cloud.origin = Eigen::Vector3f::Zero();
    point_cloud.ranges = ranges;

    return {true, point_cloud};
}

cartographer::sensor::TimedPointCloudData TimedPointCloudDataFromPCDBuilder(
    std::string file_path, double start_time) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    cartographer::sensor::TimedPointCloudData timed_pcd;
    cartographer::sensor::TimedPointCloud ranges;

    // Open the point cloud file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    auto err = pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud);

    if (err == -1) {
        return timed_pcd;
    }

    double current_time = ReadTimeFromTimestamp(file_path.substr(
        file_path.find(filename_prefix) + filename_prefix.length(),
        file_path.find(".pcd")));
    double time_delta = current_time - start_time;

    VLOG(1) << "Accessing file " << file_path << " ... ";
    VLOG(1) << "Loaded " << cloud->width * cloud->height << " data points";

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cartographer::sensor::TimedRangefinderPoint timed_rangefinder_point;
        timed_rangefinder_point.position = Eigen::Vector3f(
            cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        timed_rangefinder_point.time = 0 - i * 0.0001;

        ranges.push_back(timed_rangefinder_point);
    }

    timed_pcd.time = cartographer::common::FromUniversal(123) +
                     cartographer::common::FromSeconds(double(time_delta));
    timed_pcd.origin = Eigen::Vector3f::Zero();
    timed_pcd.ranges = ranges;

    return timed_pcd;
}

double ReadTimeFromTimestamp(std::string timestamp) {
    std::string::size_type sz;
    auto partial_time_format = time_format.substr(0, time_format.find("."));
    // Create a stream which we will use to parse the string
    std::istringstream ss(timestamp);

    // Create a tm object to store the parsed date and time.
    std::tm dt = {0};

    // Now we read from buffer using get_time manipulator
    // and formatting the input appropriately.
    ss >> std::get_time(&dt, partial_time_format.c_str());
    if (ss.fail()) {
        throw std::runtime_error(
            "timestamp cannot be parsed into a std::tm object: " + timestamp);
    }
    double timestamp_time = (double)std::mktime(&dt) - timezone;
    if (timestamp_time == -1) {
        throw std::runtime_error(
            "timestamp cannot be represented as a std::time_t object: " +
            timestamp);
    }
    auto sub_sec_index = timestamp.find(".");
    if ((sub_sec_index != std::string::npos)) {
        double sub_sec = 0;
        try {
            sub_sec = (double)std::stof(timestamp.substr(sub_sec_index), &sz);
        } catch (std::exception &e) {
            LOG(FATAL) << e.what();
            throw std::runtime_error(
                "could not extract sub seconds from timestamp: " + timestamp);
        }
        double timestamp_time_w_sub_sec = timestamp_time + sub_sec;
        return timestamp_time_w_sub_sec;
    } else {
        return timestamp_time;
    }
}

}  // namespace io
}  // namespace carto_facade
}  // namespace viam
