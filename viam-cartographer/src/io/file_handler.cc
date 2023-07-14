// This is an experimental integration of cartographer into RDK.
#include "file_handler.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdio.h>

#include <boost/filesystem.hpp>
#include <fstream>  // std::ifstream
#include <iomanip>
#include <iostream>
#include <string>
#include <regex>

#include "glog/logging.h"

namespace viam {
namespace io {

namespace fs = boost::filesystem;

const std::string MakeFilenameWithTimestamp(std::string path_to_dir,
                                            std::time_t t) {
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), time_format.c_str(),
                  std::gmtime(&t));
    return path_to_dir + "/" + "map_data_" + timestamp + ".pbstream";
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

cartographer::sensor::ImuData GetTimedIMUDataFromJSON(
    std::string file_path, double start_time) {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    cartographer::sensor::ImuData imu_data;

    double current_time = ReadTimeFromTimestamp(file_path.substr(
        file_path.find(filename_prefix) + filename_prefix.length(),
        file_path.find(".json")));
    double time_delta = current_time - start_time;

    VLOG(1) << "Accessing file " << file_path << " ... ";

    imu_data.time = cartographer::common::FromUniversal(123) +
                     cartographer::common::FromSeconds(double(time_delta));
    
    auto data = ReadDataFromJSONToArray(file_path);

    imu_data.linear_acceleration = {data[0], data[1], data[2]};
    imu_data.angular_velocity = {data[3], data[4], data[5]};

    return imu_data;
}

std::vector<std::string> ListSortedFilesInDirectory(
    std::string data_directory) {
    std::vector<std::string> file_paths;

    for (const auto& entry : fs::directory_iterator(data_directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

void RemoveFile(std::string file_path) {
    if (remove(file_path.c_str()) != 0) {
        LOG(ERROR) << "Error removing file";
    }
    return;
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
    double timestamp_time = (double)timegm(&dt);
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
        } catch (std::exception& e) {
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


std::vector<double> ReadDataFromJSONToArray(std::string filename) {
    std::ifstream in;    // Create an input file stream.
    in.open("j.json");
    if ( ! in ) {
        std::cout << "Error: Can't open the file named data.txt.\n";
        exit(1);
    }

    std::string input;
    getline(in,input);  // Get the first line from the file, if any.

    std::regex regex("\"[A-Za-z]+\":(\\d+\\.?\\d*)");
    std::smatch match;

    while (std::regex_search(input, match, regex)) {
        std::cout << match.str(1) << std::endl;
        input = match.suffix();
    }

    std::vector<double> listd;
    while (std::regex_search(input, match, regex)) {
        listd.push_back(stod(match.str(1)));
        input = match.suffix();
    }

    return listd;
    
}

}  // namespace io
}  // namespace viam
