#pragma once

#include <functional>
#include <string>
#include <vector>

#include <glim_cloud_fusion/fusion.hpp>

namespace clc {

using FusionProgressCallback = std::function<void(const glim_cloud_fusion::FusionProgress&)>;

glim_cloud_fusion::FusionResult run_fusion(
  const std::vector<glim_cloud_fusion::LidarScan>& scans,
  const glim_cloud_fusion::FusionParams& params,
  FusionProgressCallback on_progress = nullptr);

bool save_dense_pcd(const std::string& path, const glim_cloud_fusion::DensePointCloud& cloud, std::string& error);

}  // namespace clc
