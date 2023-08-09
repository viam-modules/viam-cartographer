#include "util.h"

#include <pcl/console/time.h>  // pcl::console::TicToc
#include <pcl/io/pcd_io.h>     // pcl::PCDReader
#include <pcl/point_types.h>

#include <boost/format.hpp>
#include <sstream>  // std::istringstream

namespace viam {
namespace carto_facade {
namespace util {
std::string try_file_close(std::ifstream &tempFile, std::string filename) {
    tempFile.close();
    if (tempFile.bad()) {
        return (" Failed to close ifstream object " + filename);
    }
    return "";
}

void read_and_delete_file(std::string filename, std::string *buffer) {
    std::stringstream error_forwarded;

    std::ifstream tempFile(filename);
    if (tempFile.bad()) {
        error_forwarded << "Failed to open " << filename
                        << " as ifstream object.";
        error_forwarded << try_file_close(tempFile, filename);
        throw std::runtime_error(error_forwarded.str());
    }

    std::stringstream bufferStream;
    if (bufferStream << tempFile.rdbuf()) {
        *buffer = bufferStream.str();
    } else {
        error_forwarded << "Failed to get data from " << filename
                        << " to buffer stream.";
        error_forwarded << try_file_close(tempFile, filename);
        throw std::runtime_error(error_forwarded.str());
    }

    error_forwarded << try_file_close(tempFile, filename);

    if (std::remove(filename.c_str()) != 0) {
        error_forwarded << "Failed to delete " << filename;
        throw std::runtime_error(error_forwarded.str());
    }
}

std::string pcd_header(int mapSize, bool hasColor) {
    if (hasColor)
        return str(boost::format(HEADERTEMPLATECOLOR) % mapSize % mapSize);
    else
        return str(boost::format(HEADERTEMPLATE) % mapSize % mapSize);
}

void write_float_to_buffer_in_bytes(std::string &buffer, float f) {
    auto p = (const char *)(&f);
    for (std::size_t i = 0; i < sizeof(float); ++i) {
        buffer.push_back(p[i]);
    }
}

void write_int_to_buffer_in_bytes(std::string &buffer, int d) {
    auto p = (const char *)(&d);
    for (std::size_t i = 0; i < sizeof(int); ++i) {
        buffer.push_back(p[i]);
    }
}

// based on pcl::PCDReader::read
// https://pointclouds.org/documentation/classpcl_1_1_p_c_d_reader.html#ac9451748db653fd0901a0c4b6b750552
// Doesn't implemented binary_compressed yet
// as RDK doesn't yet support that format of PCD.
// Once it does, we should add support to this function.
// https://viam.atlassian.net/browse/RSDK-3753
// returns 0 on success
// returns non zero on error
// an empty or invalid pcd is considered an error
// NOTE: The underlying pcl library does not distinguish
// between an invalid & empty pcd on version 1.13 (which is the
// version we get on mac as opposed to 1.11 which is what we get on linux).
int read_pcd(std::string pcd, pcl::PCLPointCloud2 &blob) {
    pcl::PCDReader p;

    int pcd_version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::console::TicToc tt;
    tt.tic();

    int data_type;
    unsigned int data_idx;

    std::istringstream pcd_stream(pcd);
    int res = p.readHeader(pcd_stream, blob, origin, orientation, pcd_version,

                           data_type, data_idx);
    if (res != 0) {
        LOG(ERROR) << "Failed to parse header";
        return res;
    }

    if (blob.data.size() == 0) {
        LOG(ERROR) << "Failed to parse header: pcd has no points";
        return -1;
    }

    switch (data_type) {
        // ascii
        case 0:
            VLOG(1) << "parsing as ascii";
            pcd_stream.seekg(data_idx);
            res = p.readBodyASCII(pcd_stream, blob, pcd_version);
            if (res != 0) {
                LOG(ERROR) << "Failed to parse ascii PCD body";
                return res;
            }
            break;
        // binary
        case 1: {
            VLOG(1) << "parsing as binary";
            // This block exists b/c otherwise
            // map is counterintuitively visible
            // to the rest of the case
            // statement branches
            std::size_t expected_size = data_idx + blob.data.size();
            std::size_t size = pcd.length();
            if (expected_size > size) {
                LOG(ERROR) << "Corrupted PCD file. The file is smaller "
                              "than expected! Expected: "
                           << expected_size << "actual: " << size;
                return -1;
            }
            const unsigned char *map =
                reinterpret_cast<const unsigned char *>(pcd.c_str());
            res = p.readBodyBinary(map, blob, pcd_version, false, data_idx);
            if (res != 0) {
                LOG(ERROR) << "Failed to parse binary PCD body";
                return res;
            }
        } break;
        // binary compressed
        case 2:
            LOG(ERROR) << "compressed PCDs are not currently supported";
            return -1;
            break;
        default:
            LOG(ERROR) << "PCD classified as an unsupported data type: "
                       << data_type;
            return -1;
    }
    double total_time = tt.toc();
    VLOG(1) << "[viam::carto_facade::io::read_pcd] Loaded as a "
            << (blob.is_dense ? "dense" : "non-dense") << "blob in "
            << total_time << "ms with " << blob.width * blob.height
            << "points. Available dimensions: "
            << pcl::getFieldsList(blob).c_str();
    return res;
}

std::tuple<bool, cartographer::sensor::TimedPointCloudData>
carto_lidar_reading(std::string lidar_reading,
                     int64_t lidar_reading_time_unix_milli) {
    cartographer::sensor::TimedPointCloudData point_cloud;
    cartographer::sensor::TimedPointCloud ranges;

    pcl::PCLPointCloud2 blob;

    try {
        int err = read_pcd(lidar_reading, blob);
        if (err) {
            return {false, point_cloud};
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "exception thrown during read_pcd: " << e.what();
        return {false, point_cloud};
    }
    VLOG(1) << "read_pcd succeeded";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(blob, *cloud);

    VLOG(1) << "Loaded " << cloud->width * cloud->height << " data points";

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cartographer::sensor::TimedRangefinderPoint timed_rangefinder_point;
        timed_rangefinder_point.position = Eigen::Vector3f(
            cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        // NOTE: This makes it so that each point has a time that is unique
        // within that measurement
        timed_rangefinder_point.time = 0 - i * 0.0001;

        ranges.push_back(timed_rangefinder_point);
    }

    point_cloud.time =
        cartographer::common::FromUniversal(0) +
        cartographer::common::FromMilliseconds(lidar_reading_time_unix_milli);
    point_cloud.origin = Eigen::Vector3f::Zero();
    point_cloud.ranges = ranges;

    return {true, point_cloud};
}
}  // namespace util
}  // namespace carto_facade
}  // namespace viam
