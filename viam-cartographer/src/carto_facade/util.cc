#include "util.h"

#include <boost/format.hpp>

namespace viam {
namespace carto_facade {
namespace util {
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
}  // namespace util
}  // namespace carto_facade
}  // namespace viam
