#pragma once

#include <optional>

#include <Eigen/Core>

namespace psiviz {

class DepthPicker {
 public:
  [[nodiscard]] static std::optional<Eigen::Vector3f> unproject(
      float depth01,
      int x,
      int y,
      int width,
      int height,
      float device_pixel_ratio,
      const Eigen::Matrix4f& view,
      const Eigen::Matrix4f& projection);
};

}  // namespace psiviz
