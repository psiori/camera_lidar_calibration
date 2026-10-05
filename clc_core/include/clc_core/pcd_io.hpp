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

bool load_pcd_xyzit(const std::string& path, PointXYZITCloud& cloud, std::string& error);
std::vector<std::string> list_pcd_files(const std::string& directory);

}  // namespace clc
