#include "test_helpers.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
namespace viam {
namespace utils {

// NOTE: When using this function, make sure to remove the temporary directory
// and its contents once done processing the files by calling the function
// removeTmpDirectory(tmp_dir).
boost::filesystem::path createTmpDirectoryAndAddFiles(
    std::vector<std::string> data_files, std::vector<std::string> map_files) {
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir = boost::filesystem::temp_directory_path() /
                                      boost::filesystem::unique_path();
    bool ok = boost::filesystem::create_directory(tmp_dir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " +
                                 tmp_dir.string());
    }

    // Create subdirectories
    boost::filesystem::path tmp_dir_config =
        createSubdirectory(tmp_dir, "config");
    boost::filesystem::path tmp_dir_data = createSubdirectory(tmp_dir, "data");
    boost::filesystem::path tmp_dir_map = createSubdirectory(tmp_dir, "map");

    // Add data files to "data" subdirectory
    for (std::string file : data_files) {
        boost::filesystem::ofstream ofs(tmp_dir_data / file);
    }

    // Add map files to "map" subdirectory
    for (std::string file : map_files) {
        boost::filesystem::ofstream ofs(tmp_dir_map / file);
    }

    return tmp_dir;
}

boost::filesystem::path createSubdirectory(boost::filesystem::path directory,
                                           std::string subdirectory_name) {
    boost::filesystem::path subdirectory = directory / subdirectory_name;
    bool ok = boost::filesystem::create_directory(subdirectory);
    if (!ok) {
        boost::filesystem::remove_all(directory);
        throw std::runtime_error("could not create directory: " +
                                 subdirectory.string());
    }
    return subdirectory;
}

void removeTmpDirectory(boost::filesystem::path tmp_dir) {
    boost::filesystem::remove_all(tmp_dir);
}

}  // namespace utils
}  // namespace viam
