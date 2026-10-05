#include <clc_core/fusion_service.hpp>

#include <fstream>

namespace clc {

glim_cloud_fusion::FusionResult run_fusion(
  const std::vector<glim_cloud_fusion::LidarScan>& scans,
  const glim_cloud_fusion::FusionParams& params,
  FusionProgressCallback on_progress) {
  return glim_cloud_fusion::fuse_scans(scans, params, on_progress);
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

}  // namespace clc
