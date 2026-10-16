#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <clc/linalg.hpp>

#include <clc_core/pcd_io.hpp>

namespace clc {

struct RangeImage {
  int width = 0;
  int height = 0;
  std::vector<float> range;
  std::vector<float> intensity;
};

enum class RangeImageChannel { Range, Intensity };

RangeImage build_range_image(const PointXYZITCloud& cloud);

/// Smallest finite range above no-return epsilon, for colormap slider extent; 0 if none.
float range_colormap_data_min(const std::vector<float>& range);

/// Largest finite range value, or 1.0 if empty or all non-positive.
float range_colormap_data_max(const std::vector<float>& range);

struct Polygon2f {
  std::vector<Vec2f> vertices;
};

struct MaskPolygonOp {
  Polygon2f polygon;
  /// When true, set polygon interior to 255 (keep); when false, set interior to 0 (exclude).
  bool keep_inside = false;
};

/// Rasterize polygons onto a WxH binary mask. Pixel 255 = keep, 0 = exclude.
std::vector<std::uint8_t> rasterize_mask(int width, int height, const std::vector<Polygon2f>& polygons, bool keep_inside);

/// Build mask by applying each op in order onto an initial all-keep (255) canvas.
std::vector<std::uint8_t> build_mask_from_ops(int width, int height, const std::vector<MaskPolygonOp>& ops);

bool save_mask_png(const std::string& path, int width, int height, const std::vector<std::uint8_t>& mask);
bool load_mask_png(const std::string& path, int width, int height, std::vector<std::uint8_t>& mask, std::string& error);

/// Apply WxH mask to a structured scan (`keep` per point; 0 excludes).
void apply_mask_to_scan(glim_cloud_fusion::LidarScan& scan, const std::vector<std::uint8_t>& mask);

glim_cloud_fusion::LidarScan to_lidar_scan(const PointXYZITCloud& cloud, double stamp = 0.0);

/// `to_lidar_scan` plus global mask when `mask_pixels.size() == width * height`.
glim_cloud_fusion::LidarScan to_lidar_scan_masked(
  const PointXYZITCloud& cloud,
  const std::vector<std::uint8_t>& mask_pixels,
  double stamp = 0.0);

}  // namespace clc
