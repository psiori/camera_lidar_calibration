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

  /// Nearest cloud point in screen space when the GPU depth pick misses (sparse splats, edge pixels).
  [[nodiscard]] static std::optional<Eigen::Vector3f> pickNearestOnScreen(
      const float* positions, std::uint64_t point_count, int x, int y, int width, int height, float device_pixel_ratio,
      const Eigen::Matrix4f& clip_view_projection, float max_pixel_radius);
};

}  // namespace psiviz
