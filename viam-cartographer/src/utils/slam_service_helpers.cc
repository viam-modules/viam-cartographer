#include "slam_service_helpers.h"

#include <boost/format.hpp>
#include <chrono>
#include <string>

#include "../io/file_handler.h"

namespace viam {

void exit_loop_handler(int s) {
    LOG(INFO) << "Finishing session.";
    viam::b_continue_session = false;
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

namespace utils {

ActionMode DetermineActionMode(std::string path_to_map,
                               std::chrono::seconds map_rate_sec) {
    // Check if there is an apriori map in the path_to_map directory
    std::vector<std::string> map_filenames =
        viam::io::ListSortedFilesInDirectory(path_to_map);

    // Check if there is a *.pbstream map in the path_to_map directory
    for (auto filename : map_filenames) {
        if (filename.find(".pbstream") != std::string::npos) {
            // There is an apriori map present, so we're running either in
            // updating or localization mode.
            if (map_rate_sec.count() == 0) {
                // This log line is needed by rdk integration tests.
                LOG(INFO) << "Running in localization only mode";
                return ActionMode::LOCALIZING;
            }
            // This log line is needed by rdk integration tests.
            LOG(INFO) << "Running in updating mode";
            return ActionMode::UPDATING;
        }
    }
    if (map_rate_sec.count() == 0) {
        throw std::runtime_error(
            "set to localization mode (map_rate_sec = 0) but couldn't find "
            "apriori map to localize on");
    }
    // This log line is needed by rdk integration tests.
    LOG(INFO) << "Running in mapping mode";
    return ActionMode::MAPPING;
}

std::string GetLatestMapFilename(std::string path_to_map) {
    std::string latest_map_filename;

    std::vector<std::string> map_filenames =
        viam::io::ListSortedFilesInDirectory(path_to_map);
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

std::string pcdHeader(int mapSize, bool hasColor) {
    if (hasColor)
        return str(boost::format(HEADERTEMPLATECOLOR) % mapSize % mapSize);
    else
        return str(boost::format(HEADERTEMPLATE) % mapSize % mapSize);
}

void writeFloatToBufferInBytes(std::string &buffer, float f) {
    auto p = (const char *)(&f);
    for (std::size_t i = 0; i < sizeof(float); ++i) {
        buffer.push_back(p[i]);
    }
}

void writeIntToBufferInBytes(std::string &buffer, int d) {
    auto p = (const char *)(&d);
    for (std::size_t i = 0; i < sizeof(int); ++i) {
        buffer.push_back(p[i]);
    }
}

}  // namespace utils
}  // namespace viam
