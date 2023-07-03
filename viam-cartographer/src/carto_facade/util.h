// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTO_FACADE_UTIL_H
#define VIAM_CARTO_FACADE_UTIL_H

#include <pcl/PCLPointCloud2.h>

#include <string>
#include <tuple>

#include "cartographer/sensor/timed_point_cloud_data.h"

namespace viam {
namespace carto_facade {
namespace util {
const auto HEADERTEMPLATE =
    "VERSION .7\n"
    "FIELDS x y z\n"
    // NOTE: If a float is more than 4 bytes
    // on a given platform
    // this size will be inaccurate
    "SIZE 4 4 4\n"
    "TYPE F F F\n"
    "COUNT 1 1 1\n"
    "WIDTH %d\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS %d\n"
    "DATA binary\n";

const auto HEADERTEMPLATECOLOR =
    "VERSION .7\n"
    "FIELDS x y z rgb\n"
    // NOTE: If a float is more than 4 bytes
    // on a given platform
    // this size will be inaccurate
    "SIZE 4 4 4 4\n"
    "TYPE F F F I\n"
    "COUNT 1 1 1 1\n"
    "WIDTH %d\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS %d\n"
    "DATA binary\n";
std::string pcd_header(int mapSize, bool hasColor);

void write_float_to_buffer_in_bytes(std::string &buffer, float f);

void write_int_to_buffer_in_bytes(std::string &buffer, int d);

std::tuple<bool, cartographer::sensor::TimedPointCloudData>
carto_sensor_reading(std::string sensor_reading,
                     int64_t sensor_reading_time_unix_micro);
int read_pcd(std::string pcd, pcl::PCLPointCloud2 &blob);
}  // namespace util
}  // namespace carto_facade
}  // namespace viam

#endif  // VIAM_CARTO_FACADE_UTIL_H
