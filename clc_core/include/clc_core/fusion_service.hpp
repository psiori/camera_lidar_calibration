#pragma once

#include <functional>
#include <string>
#include <vector>

#include <glim_cloud_fusion/fusion.hpp>

namespace clc {

using FusionProgressCallback = std::function<void(const glim_cloud_fusion::FusionProgress&)>;
using FusionCancellationCheck = glim_cloud_fusion::FusionCancellationCheck;

/// Fuse multiple lidar scans into one dense cloud using glim_cloud_fusion.
glim_cloud_fusion::FusionResult run_fusion(
  const std::vector<glim_cloud_fusion::LidarScan>& scans,
  const glim_cloud_fusion::FusionParams& params,
  FusionProgressCallback on_progress = nullptr,
  FusionCancellationCheck is_cancelled = nullptr);

/// Load a fused dense PCD and histogram-equalize intensities (handles legacy raw saves on disk).
bool load_fused_dense_pcd(const std::string& path, glim_cloud_fusion::DensePointCloud& cloud, std::string& error);

/// Write XYZ (+ optional intensity) PCD to disk.
bool save_dense_pcd(const std::string& path, const glim_cloud_fusion::DensePointCloud& cloud, std::string& error);

/// Write XYZ with per-point RGB; `rgb` is interleaved [0,1], length = cloud.points.size() * 3.
bool save_dense_pcd_rgb(const std::string& path, const glim_cloud_fusion::DensePointCloud& cloud,
                        const std::vector<float>& rgb, std::string& error);

}  // namespace clc
