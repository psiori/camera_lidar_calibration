#pragma once

#include <string>
#include <vector>

#include <glim_cloud_fusion/fusion.hpp>

namespace clc {

/// One structured scan loaded from disk with layout metadata.
struct PointXYZITCloud {
  int width = 0;
  int height = 0;
  double stamp = 0.0;
  glim_cloud_fusion::LidarScan scan;
};

/// Parse UTC capture time from a stem like `2026-10-01T18_46_54_574Z` (no extension).
/// Returns false if the string does not match exactly.
bool parse_utc_capture_stem_stamp(const std::string& stem, double& stamp_seconds);

/// Parse UTC scan time from a PCD basename like `2026-10-01T18_46_54_574Z.pcd`.
/// Returns false if the name does not match exactly.
bool parse_pcd_filename_stamp(const std::string& filename, double& stamp_seconds);

/// Parse UTC capture time from a camera image basename like `2026-10-01T18_46_54_574Z.jpg`.
/// Returns false if the extension is not a supported image type or the stem does not match.
bool parse_image_filename_stamp(const std::string& filename, double& stamp_seconds);

/// Load a structured XYZIT PCD; optionally require a parseable timestamp in the filename.
bool load_pcd_xyzit(const std::string& path, PointXYZITCloud& cloud, std::string& error, bool check_time = true);

/// Load a dense fused/export PCD (x, y, z, optional intensity). No scan filename timestamp required.
bool load_dense_pcd(const std::string& path, glim_cloud_fusion::DensePointCloud& cloud, std::string& error);

/// Sorted list of `.pcd` paths in a directory (non-recursive).
std::vector<std::string> list_pcd_files(const std::string& directory);

/// Order catalog paths. Multi-scan folders require UTC timestamp basenames; single-scan paths may use any name.
bool finalize_pcd_catalog_paths(std::vector<std::string>& paths, std::string& error);

}  // namespace clc
