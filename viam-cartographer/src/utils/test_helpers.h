#ifndef TEST_HELPERS_H_
#define TEST_HELPERS_H_

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace viam {
namespace utils {

// Create a unique path in the temp directory and use it to create a directory
// with three subdirectories: "config", "data", "map". Add data_files and
// map_files to the "data" and "map" subdirectories, respectively. NOTE: When
// using this function, make sure to remove the temporary directory and its
// contents once done processing the files by calling the function
// removeTmpDirectory(tmp_dir).
boost::filesystem::path createTmpDirectoryAndAddFiles(
    std::vector<std::string> data_files, std::vector<std::string> map_files);

// Create a subdirectory named subdirectory_name within the provided directory.
boost::filesystem::path createSubdirectory(boost::filesystem::path directory,
                                           std::string subdirectory_name);

// Remove the temporary directory tmp_dir and its contents
void removeTmpDirectory(boost::filesystem::path tmp_dir);

}  // namespace utils
}  // namespace viam

#endif  // TEST_HELPERS_H_
