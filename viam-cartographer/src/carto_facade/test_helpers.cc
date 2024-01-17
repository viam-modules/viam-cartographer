#include "test_helpers.h"

#include <boost/test/unit_test.hpp>
#include <fstream>

#include "util.h"

namespace viam {
namespace carto_facade {
namespace test_helpers {
std::string read_file(std::string file_path) {
    std::ifstream t(file_path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

void timed_pcd_contains(cartographer::sensor::TimedPointCloudData timed_pcd,
                        std::vector<std::vector<double>> points) {
    auto tolerance = boost::test_tools::tolerance(0.00001);
    for (int i = 0; i < points.size(); i++) {
        cartographer::sensor::TimedRangefinderPoint timed_rangefinder_point =
            timed_pcd.ranges.at(i);
        for (int j = 0; j < 3; j++) {
            BOOST_TEST(
                timed_rangefinder_point.position(j, 0) == points.at(i).at(j),
                tolerance);
        }
    }
}

std::string binary_pcd(std::vector<std::vector<double>> points) {
    std::string pcd =
        "VERSION .7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F I\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 3\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 3\n"
        "DATA binary\n";
    for (std::vector<double> point : points) {
        for (int i = 0; i < (point.size() - 1); i++) {
            viam::carto_facade::util::write_float_to_buffer_in_bytes(
                pcd, point.at(i));
        }
        viam::carto_facade::util::write_int_to_buffer_in_bytes(
            pcd, point.at(point.size() - 1));
    }
    return pcd;
}

std::string ascii_pcd(std::vector<std::vector<double>> points) {
    std::string pcd =
        "VERSION .7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F I\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 3\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 3\n"
        "DATA ascii\n";
    for (std::vector<double> point : points) {
        LOG(ERROR) << "size:" << point.size();
        for (int i = 0; i < (point.size() - 1); i++) {
            LOG(ERROR) << i;
            pcd = pcd + std::to_string(point.at(i)) + " ";
        }
        LOG(ERROR) << "last:" << point.size() - 1;
        pcd = pcd + std::to_string(point.at(point.size() - 1)) + "\n";
    }
    return pcd;
}

}  // namespace test_helpers
}  // namespace carto_facade
}  // namespace viam
