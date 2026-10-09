#include "psiviz/depth_picker.hpp"

#include <cmath>

#include <Eigen/LU>

namespace psiviz {

std::optional<Eigen::Vector3f> DepthPicker::unproject(float depth01, int x, int y, int width, int height,
                                                      float device_pixel_ratio, const Eigen::Matrix4f& view,
                                                      const Eigen::Matrix4f& projection) {
  if (depth01 <= 0.f || depth01 >= 1.f || width <= 0 || height <= 0) {
    return std::nullopt;
  }
  const float px = (static_cast<float>(x) * device_pixel_ratio + 0.5f) / (static_cast<float>(width) * device_pixel_ratio);
  const float py =
      (static_cast<float>(height) - static_cast<float>(y) * device_pixel_ratio - 0.5f) /
      (static_cast<float>(height) * device_pixel_ratio);
  const float ndc_x = px * 2.f - 1.f;
  const float ndc_y = py * 2.f - 1.f;
  const float ndc_z = depth01 * 2.f - 1.f;

  const Eigen::Matrix4f vp = projection * view;
  const Eigen::Matrix4f inv_vp = vp.inverse();
  Eigen::Vector4f clip(ndc_x, ndc_y, ndc_z, 1.f);
  Eigen::Vector4f world = inv_vp * clip;
  if (std::fabs(world.w()) < 1e-8f) {
    return std::nullopt;
  }
  world /= world.w();
  return world.head<3>();
}

}  // namespace psiviz
