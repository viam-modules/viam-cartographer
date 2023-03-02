// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
#define VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_

#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/transform/rigid_transform.h"

namespace viam {
namespace io {

// PaintSubmapSlices paints the submaps using cairo and returns
// the result in form of PaintSubmapsSlicesResult.
cartographer::io::PaintSubmapSlicesResult PaintSubmapSlices(
    const std::map<::cartographer::mapping::SubmapId,
                   cartographer::io::SubmapSlice>& submaps,
    double resolution);

// DrawPoseOnSurface draws global_pose onto the painted_slices.
void DrawPoseOnSurface(
    cartographer::io::PaintSubmapSlicesResult* painted_slices,
    cartographer::transform::Rigid3d global_pose, float resolution);

}  // namespace io
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_IO_SUBMAP_PAINTER_H_
