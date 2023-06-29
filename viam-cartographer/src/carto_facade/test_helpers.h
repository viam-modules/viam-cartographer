#ifndef VIAM_CARTO_FACADE_TEST_HELPERS_H
#define VIAM_CARTO_FACADE_TEST_HELPERS_H

#include <boost/filesystem.hpp>
#include <string>

#include "cartographer/sensor/timed_point_cloud_data.h"

namespace viam {
namespace carto_facade {
namespace test_helpers {
std::string read_file(std::string file_path);

void timed_pcd_contains(cartographer::sensor::TimedPointCloudData timed_pcd,
                        std::vector<std::vector<double>> points);

std::string binary_pcd(std::vector<std::vector<double>> points);

std::string ascii_pcd(std::vector<std::vector<double>> points);

boost::filesystem::path make_tmp_dir();
}  // namespace test_helpers
}  // namespace carto_facade
}  // namespace viam
#endif  // VIAM_CARTO_FACADE_TEST_HELPERS_H
