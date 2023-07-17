// This is an experimental integration of cartographer into RDK.
#ifndef FILE_HANDLER_H_
#define FILE_HANDLER_H_

#include <inttypes.h>

#include <chrono>
#include <ctime>
#include <ostream>
#include <ratio>
#include <string>

#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/imu_data.h"

namespace viam {
namespace io {
static const std::string filename_prefix = "_data_";
static const std::string time_format = "%Y-%m-%dT%H:%M:%S.0000Z";

// MakeFilenameWithTimestamp creates a filename for a provided sensor with a
// timestamp. The filename includes the path to the file. Does not support
// millisecond resolution.
const std::string MakeFilenameWithTimestamp(std::string path_to_dir,
                                            std::time_t t);

// ListSortedFilesInDirectory returns a list of the files in the directory
// sorted by name.
std::vector<std::string> ListSortedFilesInDirectory(std::string data_directory);

// TimedPointCloudDataFromPCDBuilder creates a TimedPointCloudData object
// from a PCD file.
cartographer::sensor::TimedPointCloudData TimedPointCloudDataFromPCDBuilder(
    std::string file_path, double start_time);

cartographer::sensor::ImuData GetTimedIMUDataFromJSON(std::string file_path, double start_time);

// RemoveFile removes the file at the provided path.
void RemoveFile(std::string);

// Converts UTC time string to a double value.
double ReadTimeFromTimestamp(std::string timestamp);

cartographer::sensor::ImuData ReadDataFromJSONToArray(std::string filename);

}  // namespace io
}  // namespace viam

#endif  // FILE_HANDLER_H_
