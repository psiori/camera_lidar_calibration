// depth_picker.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/depth_picker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace psiviz {

namespace {

struct Offset {
  int dx;
  int dy;
  int dist_sq;
};

}  // namespace

namespace {

std::uint32_t decodePickIdBytes(const unsigned char* rgba) {
  return static_cast<std::uint32_t>(rgba[0]) | (static_cast<std::uint32_t>(rgba[1]) << 8) |
         (static_cast<std::uint32_t>(rgba[2]) << 16) | (static_cast<std::uint32_t>(rgba[3]) << 24);
}

}  // namespace

std::uint32_t DepthPicker::selectPointIdFromPatch(const std::uint32_t* pixels, int patch_w, int patch_h, int center_x,
                                                  int center_y, bool flip_framebuffer_y) {
  if (!pixels || patch_w <= 0 || patch_h <= 0) {
    return 0;
  }
  const int max_r = std::max(patch_w, patch_h);
  std::vector<Offset> offsets;
  offsets.reserve(static_cast<std::size_t>((2 * max_r + 1) * (2 * max_r + 1)));
  for (int dy = -max_r; dy <= max_r; ++dy) {
    for (int dx = -max_r; dx <= max_r; ++dx) {
      const int cx = center_x + dx;
      const int cy = center_y + dy;
      if (cx < 0 || cy < 0 || cx >= patch_w || cy >= patch_h) {
        continue;
      }
      offsets.push_back({dx, dy, dx * dx + dy * dy});
    }
  }
  std::sort(offsets.begin(), offsets.end(),
            [](const Offset& a, const Offset& b) { return a.dist_sq < b.dist_sq; });

  for (const Offset& off : offsets) {
    int sx = center_x + off.dx;
    int sy = center_y + off.dy;
    if (flip_framebuffer_y) {
      sy = patch_h - 1 - sy;
    }
    const std::uint32_t id = pixels[sy * patch_w + sx];
    if (id != 0) {
      return id;
    }
  }
  return 0;
}

std::uint32_t DepthPicker::selectPointIdFromRgbaPatch(const unsigned char* pixels, int patch_w, int patch_h,
                                                      int center_x, int center_y, bool flip_framebuffer_y) {
  if (!pixels || patch_w <= 0 || patch_h <= 0) {
    return 0;
  }
  const int max_r = std::max(patch_w, patch_h);
  std::vector<Offset> offsets;
  offsets.reserve(static_cast<std::size_t>((2 * max_r + 1) * (2 * max_r + 1)));
  for (int dy = -max_r; dy <= max_r; ++dy) {
    for (int dx = -max_r; dx <= max_r; ++dx) {
      const int cx = center_x + dx;
      const int cy = center_y + dy;
      if (cx < 0 || cy < 0 || cx >= patch_w || cy >= patch_h) {
        continue;
      }
      offsets.push_back({dx, dy, dx * dx + dy * dy});
    }
  }
  std::sort(offsets.begin(), offsets.end(),
            [](const Offset& a, const Offset& b) { return a.dist_sq < b.dist_sq; });

  for (const Offset& off : offsets) {
    int sx = center_x + off.dx;
    int sy = center_y + off.dy;
    if (flip_framebuffer_y) {
      sy = patch_h - 1 - sy;
    }
    const int idx = (sy * patch_w + sx) * 4;
    const std::uint32_t id = decodePickIdBytes(pixels + idx);
    if (id != 0) {
      return id;
    }
  }
  return 0;
}

std::optional<clc::Vec3f> DepthPicker::pickNearestOnScreen(const float* positions, std::uint64_t point_count, int x,
                                                           int y, int width, int height, float device_pixel_ratio,
                                                           const clc::Mat4f& clip_view_projection, float max_pixel_radius) {
  if (!positions || point_count == 0 || width <= 0 || height <= 0 || max_pixel_radius <= 0.f) {
    return std::nullopt;
  }
  const float target_x = static_cast<float>(x) * device_pixel_ratio;
  const float target_y = static_cast<float>(y) * device_pixel_ratio;
  const float radius_sq = max_pixel_radius * max_pixel_radius;

  float best_dist_sq = std::numeric_limits<float>::infinity();
  float best_depth = std::numeric_limits<float>::infinity();
  std::uint64_t best_index = 0;
  bool found = false;

  for (std::uint64_t i = 0; i < point_count; ++i) {
    const clc::Vec4f clip =
        clc::mulMat4Vec4(clip_view_projection,
                         {positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2], 1.f});
    if (clip.w <= 1e-6f) {
      continue;
    }
    const float inv_w = 1.f / clip.w;
    const float ndc_x = clip.x * inv_w;
    const float ndc_y = clip.y * inv_w;
    const float ndc_z = clip.z * inv_w;
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
      best_index = i;
      found = true;
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return clc::Vec3f{positions[best_index * 3 + 0], positions[best_index * 3 + 1], positions[best_index * 3 + 2]};
}

}  // namespace psiviz
