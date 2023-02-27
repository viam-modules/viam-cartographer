// This is an experimental integration of cartographer into RDK.
#ifndef VIAM_CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_
#define VIAM_CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_

#include "../src/io/color.h"
#include "cairo/cairo.h"
#include "cartographer/io/color.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/transform/rigid_transform.h"

namespace viam {
namespace io {

using PoseToPixelFunction =
    std::function<Eigen::Array2i(const cartographer::transform::Rigid3d& pose)>;

// Draws the 'trajectory' with the given 'color' onto 'surface'. Function must
// translate a trajectory node's position into the pixel on 'surface'.
cartographer::io::UniqueCairoSurfacePtr DrawTrajectoryNodes(
    const cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                         cartographer::mapping::TrajectoryNode>&
        trajectory_nodes_poses,
    float resolution, cartographer::transform::Rigid3d slice_pose,
    cairo_surface_t* surface);

}  // namespace io
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_
