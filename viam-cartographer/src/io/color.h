// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTOGRAPHER_IO_COLOR_H_
#define VIAM_CARTOGRAPHER_IO_COLOR_H_

#include <array>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace viam {
namespace io {

using Uint8Color = std::array<cartographer::uint8, 3>;
using FloatColor = std::array<float, 3>;

// A function for on-demand generation of a color palette, with every two
// direct successors having large contrast.
FloatColor GetColor(int id);

}  // namespace io
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_IO_COLOR_H_
