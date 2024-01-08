#include "io.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>

namespace viam {
namespace carto_facade {
namespace io {

BOOST_AUTO_TEST_SUITE(CartoFacade_io)

BOOST_AUTO_TEST_CASE(TimedPointCloudDataFromPCDBuilder_success) {
    // Create a mini PCD file and save it in a tmp directory
    std::string filename = "rplidar_data_2022-01-01T01:00:00.0001Z.pcd";
    std::vector<std::vector<double>> points = {
        {-0.001000, 0.002000, 0.005000, 16711938},
        {0.582000, 0.012000, 0.000000, 16711938},
        {0.007000, 0.006000, 0.001000, 16711938}};
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
        for (int i = 0; i < 3; i++) {
            pcd = pcd + std::to_string(point.at(i)) + " ";
        }
        pcd = pcd + std::to_string(point.at(3)) + "\n";
    }
    // Create a unique path in the temp directory and add the PCD file
    boost::filesystem::path tmp_dir = boost::filesystem::temp_directory_path() /
                                      boost::filesystem::unique_path();
    bool ok = boost::filesystem::create_directory(tmp_dir);
    if (!ok) {
        throw std::runtime_error("could not create directory: " +
                                 tmp_dir.string());
    }
    boost::filesystem::ofstream ofs(tmp_dir / filename);
    ofs << pcd;
    ofs.close();
    // Read it in and check if the data in the TimedPointCloudData is equivalent
    // to what we had in the pcd file
    cartographer::sensor::TimedPointCloudData timed_pcd =
        TimedPointCloudDataFromPCDBuilder(tmp_dir.string() + "/" + filename, 0);

    auto tolerance = boost::test_tools::tolerance(0.00001);
    BOOST_TEST(timed_pcd.ranges.size() == points.size());
    for (int i = 0; i < points.size(); i++) {
        cartographer::sensor::TimedRangefinderPoint timed_rangefinder_point =
            timed_pcd.ranges.at(i);
        for (int j = 0; j < 3; j++) {
            BOOST_TEST(
                timed_rangefinder_point.position(j, 0) == points.at(i).at(j),
                tolerance);
        }
    }

    // Remove the temporary directory and its contents
    boost::filesystem::remove_all(tmp_dir);
}

BOOST_AUTO_TEST_CASE(ReadTimeFromTimestamp_missing_timestamp) {
    // Provide a filename with a missing timestamp
    std::string timestamp = "no-timestamp";
    const std::string message =
        "timestamp cannot be parsed into a std::tm object: " + timestamp;
    BOOST_CHECK_EXCEPTION(ReadTimeFromTimestamp(timestamp), std::runtime_error,
                          [&message](const std::runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });
}

BOOST_AUTO_TEST_CASE(ReadTimeFromTimestamp_success) {
    // Provide a filename with a timestamp
    std::time_t t = std::time(nullptr);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), time_format.c_str(),
                  std::gmtime(&t));
    std::string filename_prefix = "rplidar_data_";
    std::string filename_type = ".pcd";
    std::string filename =
        filename_prefix + std::string(timestamp) + filename_type;
    // Read the time
    std::string timestamp_str = filename.substr(
        filename.find(filename_prefix) + filename_prefix.length(),
        filename.find(filename_type));
    double filename_time = ReadTimeFromTimestamp(timestamp_str);
    auto tolerance = boost::test_tools::tolerance(0.0001);
    // Make sure the time read from the filename equals what we put into the
    // filename
    BOOST_TEST((double)t == filename_time, tolerance);
}

BOOST_AUTO_TEST_CASE(ReadTimeFromTimestamp_comparison) {
    const std::string timestamp_1 = "2022-01-01T01:00:00.0000Z";
    const std::string timestamp_2 = "2022-01-01T01:00:00.0001Z";
    const std::string timestamp_3 = "2022-01-01T01:00:01.0000Z";
    const auto time_1 = ReadTimeFromTimestamp(timestamp_1);
    const auto time_2 = ReadTimeFromTimestamp(timestamp_2);
    const auto time_3 = ReadTimeFromTimestamp(timestamp_3);
    BOOST_TEST(time_1 < time_2);
    BOOST_TEST(time_2 < time_3);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace io
}  // namespace carto_facade
}  // namespace viam
