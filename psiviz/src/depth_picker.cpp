#include "psiviz/depth_picker.hpp"

#include <cmath>
#include <limits>

#include <Eigen/LU>

namespace psiviz {

namespace {
constexpr float kDepthMiss = 0.9995f;
constexpr std::uint64_t kCpuPickMaxPoints = 2'000'000;
}  // namespace

std::optional<Eigen::Vector3f> DepthPicker::unproject(float depth01, int x, int y, int width, int height,
                                                      float device_pixel_ratio,
                                                      const Eigen::Matrix4f& clip_view_projection) {
  if (depth01 <= 0.f || depth01 >= kDepthMiss || width <= 0 || height <= 0) {
    return std::nullopt;
  }
  const float px =
      (static_cast<float>(x) * device_pixel_ratio + 0.5f) / (static_cast<float>(width) * device_pixel_ratio);
  const float py = (static_cast<float>(height) - static_cast<float>(y) * device_pixel_ratio - 0.5f) /
                   (static_cast<float>(height) * device_pixel_ratio);
  const float ndc_x = px * 2.f - 1.f;
  const float ndc_y = py * 2.f - 1.f;
  const float ndc_z = depth01 * 2.f - 1.f;

  const Eigen::Matrix4f inv_vp = clip_view_projection.inverse();
  Eigen::Vector4f clip(ndc_x, ndc_y, ndc_z, 1.f);
  Eigen::Vector4f world = inv_vp * clip;
  if (std::fabs(world.w()) < 1e-8f) {
    return std::nullopt;
  }
  world /= world.w();
  return world.head<3>();
}

std::optional<Eigen::Vector3f> DepthPicker::pickNearestOnScreen(const float* positions, std::uint64_t point_count,
                                                                int x, int y, int width, int height,
                                                                float device_pixel_ratio,
                                                                const Eigen::Matrix4f& clip_view_projection,
                                                                float max_pixel_radius) {
  if (!positions || point_count == 0 || width <= 0 || height <= 0 || max_pixel_radius <= 0.f) {
    return std::nullopt;
  }
  if (point_count > kCpuPickMaxPoints) {
    return std::nullopt;
  }

  const float target_x = static_cast<float>(x) * device_pixel_ratio;
  const float target_y = static_cast<float>(y) * device_pixel_ratio;
  const float radius_sq = max_pixel_radius * max_pixel_radius;

  float best_dist_sq = std::numeric_limits<float>::infinity();
  float best_depth = std::numeric_limits<float>::infinity();
  Eigen::Vector3f best_point = Eigen::Vector3f::Zero();
  bool found = false;

  for (std::uint64_t i = 0; i < point_count; ++i) {
    const Eigen::Vector4f world(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2], 1.f);
    const Eigen::Vector4f clip = clip_view_projection * world;
    if (clip.w() <= 1e-6f) {
      continue;
    }
    const float inv_w = 1.f / clip.w();
    const float ndc_x = clip.x() * inv_w;
    const float ndc_y = clip.y() * inv_w;
    const float ndc_z = clip.z() * inv_w;
    if (ndc_z < -1.f || ndc_z > 1.f) {
      continue;
    }
    const float sx = (ndc_x * 0.5f + 0.5f) * static_cast<float>(width) * device_pixel_ratio;
    const float sy = (1.f - (ndc_y * 0.5f + 0.5f)) * static_cast<float>(height) * device_pixel_ratio;
    const float dx = sx - target_x;
    const float dy = sy - target_y;
    const float dist_sq = dx * dx + dy * dy;
    if (dist_sq > radius_sq) {
      continue;
    }
    if (dist_sq < best_dist_sq || (dist_sq == best_dist_sq && ndc_z < best_depth)) {
      best_dist_sq = dist_sq;
      best_depth = ndc_z;
      best_point = world.head<3>();
      found = true;
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return best_point;
}

}  // namespace psiviz
