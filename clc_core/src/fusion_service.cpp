// fusion_service.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/fusion_service.hpp>

#include <algorithm>
#include <fstream>

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
  std::ofstream ofs(path);
  if (!ofs) {
    error = "failed to open " + path;
    return false;
  }
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
  ofs << "VERSION 0.7\n";
  ofs << "FIELDS x y z intensity\n";
  ofs << "SIZE 4 4 4 4\n";
  ofs << "TYPE F F F F\n";
  ofs << "COUNT 1 1 1 1\n";
  ofs << "WIDTH " << cloud.points.size() << "\n";
  ofs << "HEIGHT 1\n";
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << cloud.points.size() << "\n";
  ofs << "DATA ascii\n";
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& p = cloud.points[i];
    const double intensity = i < cloud.intensities.size() ? cloud.intensities[i] : 0.0;
    ofs << p.x() << " " << p.y() << " " << p.z() << " " << intensity << "\n";
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
  std::ofstream ofs(path);
  if (!ofs) {
    error = "failed to open " + path;
    return false;
  }
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
  ofs << "VERSION 0.7\n";
  ofs << "FIELDS x y z intensity rgb\n";
  ofs << "SIZE 4 4 4 4 4\n";
  ofs << "TYPE F F F F U\n";
  ofs << "COUNT 1 1 1 1 1\n";
  ofs << "WIDTH " << cloud.points.size() << "\n";
  ofs << "HEIGHT 1\n";
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << cloud.points.size() << "\n";
  ofs << "DATA ascii\n";
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& p = cloud.points[i];
    const double intensity = i < cloud.intensities.size() ? cloud.intensities[i] : 0.0;
    const float r = rgb[i * 3 + 0];
    const float g = rgb[i * 3 + 1];
    const float b = rgb[i * 3 + 2];
    const std::uint32_t rgb_packed =
      (static_cast<std::uint32_t>(std::clamp(r, 0.f, 1.f) * 255.f) << 16) |
      (static_cast<std::uint32_t>(std::clamp(g, 0.f, 1.f) * 255.f) << 8) |
      static_cast<std::uint32_t>(std::clamp(b, 0.f, 1.f) * 255.f);
    ofs << p.x() << " " << p.y() << " " << p.z() << " " << intensity << " " << rgb_packed << "\n";
  }
  return true;
}

}  // namespace clc
