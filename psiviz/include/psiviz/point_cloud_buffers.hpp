#pragma once

#include <cstdint>
#include <vector>

namespace psiviz {

/// CPU-side point cloud payload uploaded to the GPU each frame when dirty.
struct PointCloudData {
  /// World-space positions as interleaved x,y,z (length = 3 * point_count).
  std::vector<float> positions;
  /// Per-point scalar used for colormap blending (length = point_count).
  std::vector<float> scalars;
  /// Per-point RGB in [0,1], interleaved (length = 3 * point_count).
  std::vector<float> rgb;
  /// Number of points; must match sizes of the vectors above.
  std::uint64_t point_count{0};
};

}  // namespace psiviz
