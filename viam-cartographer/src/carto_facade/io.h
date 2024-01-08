// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTO_FACADE_IO_H
#define VIAM_CARTO_FACADE_IO_H

#include <inttypes.h>

#include <chrono>
#include <ctime>
#include <ostream>
#include <ratio>
#include <string>

#include "cartographer/sensor/timed_point_cloud_data.h"

namespace viam {
namespace carto_facade {
namespace io {
static const std::string filename_prefix = "_data_";
static const std::string time_format = "%Y-%m-%dT%H:%M:%S.0000Z";

// TimedPointCloudDataFromPCDBuilder creates a TimedPointCloudData object
// from a PCD file.
cartographer::sensor::TimedPointCloudData TimedPointCloudDataFromPCDBuilder(
    std::string file_path, double start_time);

// Converts UTC time string to a double value.
double ReadTimeFromTimestamp(std::string timestamp);

}  // namespace io
}  // namespace carto_facade
}  // namespace viam

#endif  // VIAM_CARTO_FACADE_IO_H
