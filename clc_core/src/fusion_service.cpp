// fusion_service.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/fusion_service.hpp>

#include <algorithm>
#include <cstdint>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/// PCD point layout for colored dense clouds (x, y, z, intensity, packed rgb).
struct EIGEN_ALIGN16 ClcPointXYZIRGB {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t rgb;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  ClcPointXYZIRGB,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, rgb, rgb))

namespace {

std::uint32_t pack_rgb_floats(float r, float g, float b) {
  const auto to_byte = [](float channel) {
    return static_cast<std::uint32_t>(std::clamp(channel, 0.f, 1.f) * 255.f);
  };
  return (to_byte(r) << 16) | (to_byte(g) << 8) | to_byte(b);
}

}  // namespace

namespace clc {

glim_cloud_fusion::FusionResult run_fusion(
  const std::vector<glim_cloud_fusion::LidarScan>& scans,
  const glim_cloud_fusion::FusionParams& params,
  FusionProgressCallback on_progress,
  FusionCancellationCheck is_cancelled) {
  return glim_cloud_fusion::fuse_scans(scans, params, on_progress, is_cancelled);
}

bool save_dense_pcd(const std::string& path, const glim_cloud_fusion::DensePointCloud& cloud, std::string& error) {
  if (cloud.points.empty()) {
    error = "empty cloud";
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl_cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;
  pcl_cloud.points.resize(cloud.points.size());
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& p = cloud.points[i];
    pcl_cloud.points[i].x = static_cast<float>(p.x());
    pcl_cloud.points[i].y = static_cast<float>(p.y());
    pcl_cloud.points[i].z = static_cast<float>(p.z());
    pcl_cloud.points[i].intensity =
      i < cloud.intensities.size() ? static_cast<float>(cloud.intensities[i]) : 0.f;
  }
  if (pcl::io::savePCDFileBinaryCompressed(path, pcl_cloud) != 0) {
    error = "failed to write PCD: " + path;
    return false;
  }
  return true;
}

bool save_dense_pcd_rgb(const std::string& path, const glim_cloud_fusion::DensePointCloud& cloud,
                        const std::vector<float>& rgb, std::string& error) {
  if (cloud.points.empty()) {
    error = "empty cloud";
    return false;
  }
  if (rgb.size() != cloud.points.size() * 3) {
    error = "rgb buffer size mismatch";
    return false;
  }
  pcl::PointCloud<ClcPointXYZIRGB> pcl_cloud;
  pcl_cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;
  pcl_cloud.points.resize(cloud.points.size());
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& p = cloud.points[i];
    auto& out = pcl_cloud.points[i];
    out.x = static_cast<float>(p.x());
    out.y = static_cast<float>(p.y());
    out.z = static_cast<float>(p.z());
    out.intensity = i < cloud.intensities.size() ? static_cast<float>(cloud.intensities[i]) : 0.f;
    out.rgb = pack_rgb_floats(rgb[i * 3 + 0], rgb[i * 3 + 1], rgb[i * 3 + 2]);
  }
  if (pcl::io::savePCDFileBinaryCompressed(path, pcl_cloud) != 0) {
    error = "failed to write PCD: " + path;
    return false;
  }
  return true;
}

}  // namespace clc
