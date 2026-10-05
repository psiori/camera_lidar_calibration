#include <clc_core/pcd_io.hpp>

#include <filesystem>
#include <fstream>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, time, time))

namespace clc {

bool load_pcd_xyzit(const std::string& path, PointXYZITCloud& cloud, std::string& error) {
  pcl::PointCloud<PointXYZIT>::Ptr pcl_cloud(new pcl::PointCloud<PointXYZIT>);
  if (pcl::io::loadPCDFile<PointXYZIT>(path, *pcl_cloud) != 0) {
    error = "failed to load PCD: " + path;
    return false;
  }
  cloud.width = static_cast<int>(pcl_cloud->width);
  cloud.height = static_cast<int>(pcl_cloud->height);
  cloud.scan.points.clear();
  cloud.scan.times.clear();
  cloud.scan.intensities.clear();
  cloud.scan.width = cloud.width;
  cloud.scan.height = cloud.height;
  for (const auto& p : *pcl_cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    cloud.scan.points.emplace_back(p.x, p.y, p.z, 1.0);
    cloud.scan.times.push_back(static_cast<double>(p.time));
    cloud.scan.intensities.push_back(p.intensity);
  }
  if (cloud.scan.points.empty()) {
    error = "PCD contains no valid points";
    return false;
  }
  return true;
}

std::vector<std::string> list_pcd_files(const std::string& directory) {
  std::vector<std::string> files;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

}  // namespace clc
