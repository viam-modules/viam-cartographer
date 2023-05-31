// This is an experimental integration of cartographer into RDK.
#ifndef SLAM_SERVICE_HELPER_FUNCTIONS_H_
#define SLAM_SERVICE_HELPER_FUNCTIONS_H_

#include <chrono>
#include <iostream>
#include <string>
#include "glog/logging.h"

namespace viam {

enum class ActionMode { MAPPING, LOCALIZING, UPDATING };
std::ostream& operator<<(std::ostream& os, const ActionMode& action_mode);

extern std::atomic<bool> b_continue_session;

void exit_loop_handler(int s);

namespace utils {

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

// DetermineActionMode determines the action mode the slam service runs in,
// which is either mapping, updating, or localizing.
ActionMode DetermineActionMode(std::string path_to_map,
                               std::chrono::seconds map_rate_sec);

// GetLatestMapFilename gets the latest map filename that is
// located in path_to_map.
std::string GetLatestMapFilename(std::string path_to_map);

// Casts the float f to a pointer of unsigned 8-bit bytes,
// iterates through all the 8-bit bytes of f, and
// writes each 8-bit byte to the buffer.
void writeFloatToBufferInBytes(std::string& buffer, float f);

// Casts the integer d to a pointer of unsigned 8-bit bytes,
// iterates through all the 8-bit bytes of d, and
// writes each 8-bit byte to the buffer.
void writeIntToBufferInBytes(std::string& buffer, int d);

// Applies the mapSize to the header template and
// returns the pcd header as a string.
std::string pcdHeader(int mapSize, bool hasColor);

}  // namespace utils
}  // namespace viam

#endif  // SLAM_SERVICE_HELPER_FUNCTIONS_H_
