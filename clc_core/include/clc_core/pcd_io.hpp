#pragma once

#include <string>
#include <vector>

#include <glim_cloud_fusion/fusion.hpp>

namespace clc {

struct PointXYZITCloud {
  int width = 0;
  int height = 0;
  double stamp = 0.0;
  glim_cloud_fusion::LidarScan scan;
};

/// Parse UTC scan time from a PCD basename like `2026-10-01T18_46_54_574Z.pcd`.
/// Returns false if the name does not match exactly.
bool parse_pcd_filename_stamp(const std::string& filename, double& stamp_seconds);

bool load_pcd_xyzit(const std::string& path, PointXYZITCloud& cloud, std::string& error, bool check_time = true);

/// Load a dense fused/export PCD (x, y, z, optional intensity). No scan filename timestamp required.
bool load_dense_pcd(const std::string& path, glim_cloud_fusion::DensePointCloud& cloud, std::string& error);

std::vector<std::string> list_pcd_files(const std::string& directory);

/// Order catalog paths. Multi-scan folders require UTC timestamp basenames; single-scan paths may use any name.
bool finalize_pcd_catalog_paths(std::vector<std::string>& paths, std::string& error);

}  // namespace clc
