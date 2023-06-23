#include "util.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/test/unit_test.hpp>
#include <cstdio>
#include <exception>
#include <iostream>
#include <string>

#include "io.h"
#include "test_helpers.h"
namespace help = viam::carto_facade::test_helpers;

namespace viam {
namespace carto_facade {
namespace util {

BOOST_AUTO_TEST_SUITE(CartoFacade_io_demo, *boost::unit_test::disabled())

BOOST_AUTO_TEST_CASE(carto_sensor_reading_empty_failure) {
  auto [success, _] = carto_sensor_reading("", 16409988000001121);
  BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_corrupt_header_ascii_failure) {
  std::vector<std::vector<double>> points = {
      {-0.001000, 0.002000, 0.005000, 16711938},
      {0.582000, 0.012000, 0.000000, 16711938},
      {0.007000, 0.006000, 0.001000, 16711938}};

  auto [success, _] = carto_sensor_reading(
      help::ascii_pcd(points).substr(1, std::string::npos), 16409988000001121);
  BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_corrupt_header_binary_failure) {
  std::vector<std::vector<double>> points = {
      {-0.001000, 0.002000, 0.005000, 16711938},
      {0.582000, 0.012000, 0.000000, 16711938},
      {0.007000, 0.006000, 0.001000, 16711938}};

  auto [success, _] = carto_sensor_reading(
      help::binary_pcd(points).substr(1, std::string::npos), 16409988000001121);
  BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_too_few_points_ascii_failure) {
  std::vector<std::vector<double>> too_few_points = {
      {0.007000, 0.006000, 0.001000, 16711938}};

  auto [success, _] =
      carto_sensor_reading(help::ascii_pcd(too_few_points), 16409988000001121);
  BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_too_few_points_binary_failure) {
  std::vector<std::vector<double>> too_few_points = {
      {0.007000, 0.006000, 0.001000, 16711938}};

  auto [success, _] =
      carto_sensor_reading(help::binary_pcd(too_few_points), 16409988000001121);
  BOOST_TEST(!success);
}

// The lib we use will parse as many points as the header specifies
// and ignore any others
BOOST_AUTO_TEST_CASE(carto_sensor_reading_too_many_points_ascii_success) {
  std::vector<std::vector<double>> too_many_points = {
      {-0.001000, 0.002000, 0.005000, 16711938},
      {0.582000, 0.012000, 0.000000, 16711938},
      {0.007000, 0.006000, 0.001000, 16711938},
      {0.001000, 0.102000, 0.105000, 16711938}};

  auto [success, timed_pcd] =
      carto_sensor_reading(help::ascii_pcd(too_many_points), 16409988000001121);
  BOOST_TEST(success);
  BOOST_TEST(timed_pcd.ranges.size() == 3);

  std::vector<std::vector<double>> points(std::begin(too_many_points), std::end(too_many_points) - 1);
  help::timed_pcd_contains(timed_pcd, points);
  BOOST_TEST(timed_pcd.origin == Eigen::Vector3f::Zero());
  BOOST_TEST(timed_pcd.time ==
             cartographer::common::FromUniversal(16409988000001121));
}

// The lib we use will parse as many points as the header specifies
// and ignore any others
BOOST_AUTO_TEST_CASE(carto_sensor_reading_too_many_points_binary_success) {
  std::vector<std::vector<double>> too_many_points = {
      {-0.001000, 0.002000, 0.005000, 16711938},
      {0.582000, 0.012000, 0.000000, 16711938},
      {0.007000, 0.006000, 0.001000, 16711938},
      {0.001000, 0.102000, 0.105000, 16711938}};

  auto [success, timed_pcd] = carto_sensor_reading(
      help::binary_pcd(too_many_points), 16409988000001121);
  BOOST_TEST(success);
  BOOST_TEST(timed_pcd.ranges.size() == 3);

  std::vector<std::vector<double>> points(std::begin(too_many_points), std::end(too_many_points) - 1);
  help::timed_pcd_contains(timed_pcd, points);
  BOOST_TEST(timed_pcd.origin == Eigen::Vector3f::Zero());
  BOOST_TEST(timed_pcd.time ==
             cartographer::common::FromUniversal(16409988000001121));
}

// The lib we use will parse as many points as the header specifies
// and ignore any others
BOOST_AUTO_TEST_CASE(carto_sensor_reading_wrong_shape_ascii_failure) {
  std::vector<std::vector<double>> wrong_point_shape = {
      {0.007000, 0.006000, 0.001000},
      {0.007000, 0.006000, 0.001000},
      {0.007000, 0.006000, 0.001000}};
  auto data = help::ascii_pcd(wrong_point_shape);
  BOOST_TEST_CHECKPOINT("checkpoint_message");
  auto [success, _] = carto_sensor_reading(data, 16409988000001121);
  BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_wrong_shape_binary_failure) {
  std::vector<std::vector<double>> wrong_point_shape = {
      {0.007000, 0.006000, 0.001000},
      {0.007000, 0.006000, 0.001000},
      {0.007000, 0.006000, 0.001000}};

  auto [success, _] = carto_sensor_reading(help::binary_pcd(wrong_point_shape),
                                           16409988000001121);
  BOOST_TEST(!success);
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_binary_success) {
  // Create a mini PCD file and save it in a tmp directory
  std::string filename = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
  std::vector<std::vector<double>> points = {
      {-0.001000, 0.002000, 0.005000, 16711938},
      {0.582000, 0.012000, 0.000000, 16711938},
      {0.007000, 0.006000, 0.001000, 16711938}};
  std::string pcd = help::binary_pcd(points);
  boost::filesystem::path tmp_dir = help::make_tmp_dir();
  // Create a unique path in the temp directory and add the PCD file
  boost::filesystem::ofstream ofs(tmp_dir / filename);
  ofs << pcd;
  ofs.close();
  // Read it in and check if the data in the TimedPointCloudData is equivalent
  // to what we had in the pcd file
  cartographer::sensor::TimedPointCloudData timed_pcd =
      viam::carto_facade::io::TimedPointCloudDataFromPCDBuilder(
          tmp_dir.string() + "/" + filename, 0);

  BOOST_TEST(timed_pcd.ranges.size() == points.size());
  help::timed_pcd_contains(timed_pcd, points);
  BOOST_TEST(timed_pcd.origin == Eigen::Vector3f::Zero());
  BOOST_TEST(timed_pcd.time ==
             cartographer::common::FromUniversal(16409988000001121));
  // Remove the temporary directory and its contents
  boost::filesystem::remove_all(tmp_dir);

  auto [success, timed_pcd_from_string] =
      carto_sensor_reading(pcd, 16409988000001121);
  BOOST_TEST(success);
  BOOST_TEST(timed_pcd_from_string.ranges.size() == points.size());
  help::timed_pcd_contains(timed_pcd_from_string, points);
  BOOST_TEST(timed_pcd_from_string.origin == Eigen::Vector3f::Zero());
  BOOST_TEST(timed_pcd_from_string.time ==
             cartographer::common::FromUniversal(16409988000001121));
}

BOOST_AUTO_TEST_CASE(carto_sensor_reading_ascii_success) {
  // Create a mini PCD file and save it in a tmp directory
  std::string filename = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
  std::vector<std::vector<double>> points = {
      {-0.001000, 0.002000, 0.005000, 16711938},
      {0.582000, 0.012000, 0.000000, 16711938},
      {0.007000, 0.006000, 0.001000, 16711938}};
  std::string pcd = help::ascii_pcd(points);
  // Create a unique path in the temp directory and add the PCD file
  boost::filesystem::path tmp_dir = help::make_tmp_dir();
  boost::filesystem::ofstream ofs(tmp_dir / filename);
  ofs << pcd;
  ofs.close();
  // Read it in and check if the data in the TimedPointCloudData is equivalent
  // to what we had in the pcd file
  cartographer::sensor::TimedPointCloudData timed_pcd =
      viam::carto_facade::io::TimedPointCloudDataFromPCDBuilder(
          tmp_dir.string() + "/" + filename, 0);

  BOOST_TEST(timed_pcd.ranges.size() == points.size());
  help::timed_pcd_contains(timed_pcd, points);
  BOOST_TEST(timed_pcd.origin == Eigen::Vector3f::Zero());
  BOOST_TEST(timed_pcd.time ==
             cartographer::common::FromUniversal(16409988000001121));
  // Remove the temporary directory and its contents
  boost::filesystem::remove_all(tmp_dir);

  auto [success, timed_pcd_from_string] =
      carto_sensor_reading(pcd, 16409988000001121);
  BOOST_TEST(success);
  BOOST_TEST(timed_pcd_from_string.ranges.size() == points.size());
  help::timed_pcd_contains(timed_pcd_from_string, points);
  BOOST_TEST(timed_pcd_from_string.origin == Eigen::Vector3f::Zero());
  BOOST_TEST(timed_pcd_from_string.time ==
             cartographer::common::FromUniversal(16409988000001121));
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace util
} // namespace carto_facade
} // namespace viam
