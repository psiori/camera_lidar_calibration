#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

namespace psiviz {

class DepthPicker {
 public:
  /// Nearest cloud point in screen space (CPU fallback / tests).
  [[nodiscard]] static std::optional<Eigen::Vector3f> pickNearestOnScreen(
      const float* positions, std::uint64_t point_count, int x, int y, int width, int height, float device_pixel_ratio,
      const Eigen::Matrix4f& clip_view_projection, float max_pixel_radius);

  /// `pixels` is row-major R32UI patch; `center_x/y` are in patch coordinates (Qt top-left).
  [[nodiscard]] static std::uint32_t selectPointIdFromPatch(const std::uint32_t* pixels, int patch_w, int patch_h,
                                                              int center_x, int center_y, bool flip_framebuffer_y);

  /// RGBA8 patch with point ids packed in little-endian byte order (see pick.frag).
  [[nodiscard]] static std::uint32_t selectPointIdFromRgbaPatch(const unsigned char* pixels, int patch_w, int patch_h,
                                                                int center_x, int center_y, bool flip_framebuffer_y);
};

}  // namespace psiviz
