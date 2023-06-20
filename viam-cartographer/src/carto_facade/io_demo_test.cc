#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/test/unit_test.hpp>
#include <cstdio>
#include <exception>
#include <iostream>
#include <string>

#include "io.h"
#include "util.h"

namespace viam {
namespace carto_facade {
namespace io {

BOOST_AUTO_TEST_SUITE(CartoFacade_io_demo, *boost::unit_test::disabled())

/*
 * 1. Show how you can read a PCD file into state cartographer can understand
 * 2. Show that you can get a string of that PCD file & get the same internal
 * state
 * 3. Remove the reliance on start_time
 */

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

boost::filesystem::path make_tmp_dir() {
    boost::filesystem::path tmp_dir = boost::filesystem::temp_directory_path() /
                                      boost::filesystem::unique_path();
    bool ok = boost::filesystem::create_directory(tmp_dir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " +
                                 tmp_dir.string());
    }
    return tmp_dir;
}

BOOST_AUTO_TEST_CASE(ToSensorData_binary_success) {
    // Create a mini PCD file and save it in a tmp directory
    std::string filename = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};
    std::string pcd = binary_pcd(points);
    boost::filesystem::path tmp_dir = make_tmp_dir();
    // Create a unique path in the temp directory and add the PCD file
    boost::filesystem::ofstream ofs(tmp_dir / filename);
    ofs << pcd;
    ofs.close();
    // Read it in and check if the data in the TimedPointCloudData is equivalent
    // to what we had in the pcd file
    cartographer::sensor::TimedPointCloudData timed_pcd =
        TimedPointCloudDataFromPCDBuilder(tmp_dir.string() + "/" + filename, 0);

    BOOST_TEST(timed_pcd.ranges.size() == points.size());
    timed_pcd_contains(timed_pcd, points);
    BOOST_TEST(timed_pcd.origin == Eigen::Vector3f::Zero());
    BOOST_TEST(timed_pcd.time ==
               cartographer::common::FromUniversal(16409988000001121));
    // Remove the temporary directory and its contents
    boost::filesystem::remove_all(tmp_dir);

    auto [success, timed_pcd_from_string] =
        ToSensorData(pcd, 16409988000001121);
    BOOST_TEST(success);
    BOOST_TEST(timed_pcd_from_string.ranges.size() == points.size());
    timed_pcd_contains(timed_pcd_from_string, points);
    BOOST_TEST(timed_pcd_from_string.origin == Eigen::Vector3f::Zero());
    BOOST_TEST(timed_pcd_from_string.time ==
               cartographer::common::FromUniversal(16409988000001121));
}

BOOST_AUTO_TEST_CASE(ToSensorData_empty_failure) {
    auto [success, _] = ToSensorData("", 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(ToSensorData_corrupt_header_ascii_failure) {
    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};

    auto [success, _] = ToSensorData(
        ascii_pcd(points).substr(1, std::string::npos), 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(ToSensorData_corrupt_header_binary_failure) {
    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};

    auto [success, _] = ToSensorData(
        binary_pcd(points).substr(1, std::string::npos), 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(ToSensorData_too_few_points_ascii_failure) {
    std::vector<std::vector<double>> too_few_points = {
        {0.007000, 0.006000, 0.001000, 16711938}};

    auto [success, _] =
        ToSensorData(ascii_pcd(too_few_points), 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(ToSensorData_too_few_points_binary_failure) {
    std::vector<std::vector<double>> too_few_points = {
        {0.007000, 0.006000, 0.001000, 16711938}};

    auto [success, _] =
        ToSensorData(binary_pcd(too_few_points), 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(ToSensorData_too_many_points_ascii_failure) {
    std::vector<std::vector<double>> too_many_points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938},
        {0.001000, 0.102000, 0.105000, 16711938}};

    auto [success, timed_pcd_from_string] =
        ToSensorData(ascii_pcd(too_many_points), 16409988000001121);
    BOOST_TEST(timed_pcd_from_string.ranges.size() == 3);
    BOOST_TEST(success);
}

// The lib we use will parse as many points as the header specifies
// and ignore any others
BOOST_AUTO_TEST_CASE(ToSensorData_too_many_points_binary_success) {
    std::vector<std::vector<double>> too_many_points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938},
        {0.001000, 0.102000, 0.105000, 16711938}};

    auto [success, timed_pcd_from_string] =
        ToSensorData(binary_pcd(too_many_points), 16409988000001121);
    BOOST_TEST(timed_pcd_from_string.ranges.size() == 3);
    BOOST_TEST(success);
}

// The lib we use will parse as many points as the header specifies
// and ignore any others
BOOST_AUTO_TEST_CASE(ToSensorData_wrong_shape_ascii_failure) {
    std::vector<std::vector<double>> wrong_point_shape = {
        {0.007000, 0.006000, 0.001000},
        {0.007000, 0.006000, 0.001000},
        {0.007000, 0.006000, 0.001000}};
    auto data = ascii_pcd(wrong_point_shape);
    BOOST_TEST_CHECKPOINT("checkpoint_message");
    auto [success, _] = ToSensorData(data, 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(ToSensorData_wrong_shape_binary_failure) {
    std::vector<std::vector<double>> wrong_point_shape = {
        {0.007000, 0.006000, 0.001000},
        {0.007000, 0.006000, 0.001000},
        {0.007000, 0.006000, 0.001000}};

    auto [success, _] =
        ToSensorData(binary_pcd(wrong_point_shape), 16409988000001121);
    BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(TimedPointCloudDataFromPCDBuilder_ascii_success) {
    // Create a mini PCD file and save it in a tmp directory
    std::string filename = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};
    std::string pcd = ascii_pcd(points);
    // Create a unique path in the temp directory and add the PCD file
    boost::filesystem::path tmp_dir = make_tmp_dir();
    boost::filesystem::ofstream ofs(tmp_dir / filename);
    ofs << pcd;
    ofs.close();
    // Read it in and check if the data in the TimedPointCloudData is equivalent
    // to what we had in the pcd file
    cartographer::sensor::TimedPointCloudData timed_pcd =
        TimedPointCloudDataFromPCDBuilder(tmp_dir.string() + "/" + filename, 0);

    BOOST_TEST(timed_pcd.ranges.size() == points.size());
    timed_pcd_contains(timed_pcd, points);
    BOOST_TEST(timed_pcd.origin == Eigen::Vector3f::Zero());
    BOOST_TEST(timed_pcd.time ==
               cartographer::common::FromUniversal(16409988000001121));
    // Remove the temporary directory and its contents
    boost::filesystem::remove_all(tmp_dir);

    auto [success, timed_pcd_from_string] =
        ToSensorData(pcd, 16409988000001121);
    BOOST_TEST(success);
    BOOST_TEST(timed_pcd_from_string.ranges.size() == points.size());
    timed_pcd_contains(timed_pcd_from_string, points);
    BOOST_TEST(timed_pcd_from_string.origin == Eigen::Vector3f::Zero());
    BOOST_TEST(timed_pcd_from_string.time ==
               cartographer::common::FromUniversal(16409988000001121));
}

BOOST_AUTO_TEST_CASE(TimedPointCloudDataFromPCDBuilder_binary_ascii_compare) {
    // Create a mini PCD file and save it in a tmp directory
    std::string filename_ascii = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
    std::string filename_binary = "rplidar_data_2022-01-01T01:00:00.0002Z.pcd";
    boost::filesystem::path tmp_dir = make_tmp_dir();

    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};
    std::string pcd_ascii = ascii_pcd(points);
    {
        boost::filesystem::ofstream ofs(tmp_dir / filename_ascii);
        ofs << pcd_ascii;
        ofs.close();
    }

    std::string pcd_binary = binary_pcd(points);
    {
        boost::filesystem::ofstream ofs(tmp_dir / filename_binary);
        ofs << pcd_binary;
        ofs.close();
    }
    // Read it in and check if the data in the TimedPointCloudData is
    // equivalent
    // to what we had in the pcd file
    cartographer::sensor::TimedPointCloudData timed_pcd_ascii =
        TimedPointCloudDataFromPCDBuilder(
            tmp_dir.string() + "/" + filename_ascii, 0);
    cartographer::sensor::TimedPointCloudData timed_pcd_binary =
        TimedPointCloudDataFromPCDBuilder(
            tmp_dir.string() + "/" + filename_binary, 0);

    BOOST_TEST(timed_pcd_ascii.ranges.size() == points.size());
    timed_pcd_contains(timed_pcd_ascii, points);
    BOOST_TEST(timed_pcd_ascii.origin == Eigen::Vector3f::Zero());
    BOOST_TEST(timed_pcd_ascii.time ==
               cartographer::common::FromUniversal(16409988000001121));

    BOOST_TEST(timed_pcd_binary.ranges.size() == points.size());
    timed_pcd_contains(timed_pcd_binary, points);
    BOOST_TEST(timed_pcd_binary.origin == Eigen::Vector3f::Zero());
    BOOST_TEST(timed_pcd_binary.time ==
               cartographer::common::FromUniversal(16409988000002123));

    // Remove the temporary directory and its contents
    boost::filesystem::remove_all(tmp_dir);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace io
}  // namespace carto_facade
}  // namespace viam
