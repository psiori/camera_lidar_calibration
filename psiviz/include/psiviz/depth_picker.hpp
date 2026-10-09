#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

namespace psiviz {

class DepthPicker {
 public:
  /// `clip_view_projection` must match the shader MVP (clip_corr * projection * view).
  [[nodiscard]] static std::optional<Eigen::Vector3f> unproject(
      float depth01, int x, int y, int width, int height, float device_pixel_ratio,
      const Eigen::Matrix4f& clip_view_projection);

  /// Screen-space nearest point when depth-buffer pick fails or is ambiguous.
  [[nodiscard]] static std::optional<Eigen::Vector3f> pickNearestOnScreen(
      const float* positions, std::uint64_t point_count, int x, int y, int width, int height, float device_pixel_ratio,
      const Eigen::Matrix4f& clip_view_projection, float max_pixel_radius);
};

}  // namespace psiviz
