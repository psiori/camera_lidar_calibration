#pragma once

#include <string>
#include <vector>

namespace clc {

/// Result of scanning a folder (or single file) for structured lidar PCDs.
struct PcdDirectoryScan {
  std::vector<std::string> paths;
  int reference_width = 0;
  int reference_height = 0;
  std::string error;
};

/// List `.pcd` files in a directory and validate structured width/height against reference (0 = set from first file).
PcdDirectoryScan scan_pcd_directory(const std::string& directory, int reference_width, int reference_height);

/// Resolve `path` as a single `.pcd` file or a directory of scans (same validation as scan_pcd_directory).
PcdDirectoryScan scan_pcd_source(const std::string& path, int reference_width, int reference_height);

/// Read WIDTH/HEIGHT fields from a structured PCD header without loading all points.
bool read_pcd_dimensions(const std::string& path, int& width, int& height, std::string& error);

}  // namespace clc
