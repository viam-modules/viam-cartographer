// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_IO_IMAGE_H_
#define VIAM_IO_IMAGE_H_

#include <cstdint>
#include <vector>

#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/color.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/io/points_batch.h"

namespace viam {
namespace io {

class Image : public cartographer::io::Image {
   public:
    using cartographer::io::Image::Image;
    std::string WriteJpegToString(int quality);
};

}  // namespace io
}  // namespace viam

#endif  // VIAM_IO_IMAGE_H_
