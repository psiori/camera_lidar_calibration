#pragma once

#include <string>
#include <vector>

namespace clc {

struct PcdDirectoryScan {
  std::vector<std::string> paths;
  int reference_width = 0;
  int reference_height = 0;
  std::string error;
};

/// List `.pcd` files and validate structured width/height against reference (0 = set from first file).
PcdDirectoryScan scan_pcd_directory(const std::string& directory, int reference_width, int reference_height);

bool read_pcd_dimensions(const std::string& path, int& width, int& height, std::string& error);

}  // namespace clc
