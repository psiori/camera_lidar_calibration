#include <clc_core/pcd_io.hpp>

#include <cctype>
#include <filesystem>
#include <fstream>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


/** @brief  Smaller point type with 64bit timestamp and just 8 bit intensity
 */
struct _PointXYZIT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY_8U;
  std::uint16_t ambient;
  std::uint64_t time;
  PCL_MAKE_ALIGNED_OPERATOR_NEW;
};

struct PointXYZIT : public _PointXYZIT {
  inline constexpr PointXYZIT(const _PointXYZIT& p) :
      PointXYZIT(p.x, p.y, p.z, p.intensity, p.ambient, p.time) {
  }

  inline constexpr PointXYZIT() : PointXYZIT(0.f, 0.f, 0.f, 0, 0, 0) {
  }

  inline constexpr PointXYZIT(float _x,
                              float _y,
                              float _z,
                              std::uint8_t _intensity,
                              std::uint16_t _ambient,
                              std::uint64_t _time) :
      _PointXYZIT({{{_x, _y, _z, 1.f}}, {_intensity}, _ambient, _time}) {
  }
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
      std::uint16_t, ambient, ambient)(std::uint64_t, time, time))




namespace clc {

namespace {

bool path_has_pcd_extension(const std::filesystem::path& path) {
  std::string ext = path.extension().string();
  for (char& c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return ext == ".pcd";
}

}  // namespace

bool load_pcd_xyzit(const std::string& path, PointXYZITCloud& cloud, std::string& error) {
  if (!path_has_pcd_extension(path)) {
    error = "not a PCD file: " + path;
    return false;
  }
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
  cloud.scan.keep.clear();
  cloud.scan.width = cloud.width;
  cloud.scan.height = cloud.height;
  cloud.scan.points.reserve(pcl_cloud->size());
  bool any_valid = false;
  for (const auto& p : *pcl_cloud) {
    const bool valid = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    cloud.scan.points.emplace_back(p.x, p.y, p.z, 1.0);
    cloud.scan.times.push_back(static_cast<double>(p.time));
    cloud.scan.intensities.push_back(p.intensity);
    cloud.scan.keep.push_back(valid);
    any_valid = any_valid || valid;
  }
  if (!any_valid) {
    error = "PCD contains no valid points";
    return false;
  }
  return true;
}

std::vector<std::string> list_pcd_files(const std::string& directory) {
  std::vector<std::string> files;
  std::error_code ec;
  const std::filesystem::path root(directory);
  if (!std::filesystem::is_directory(root, ec)) {
    return files;
  }
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root, ec)) {
    if (ec) {
      break;
    }
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto& path = entry.path();
    const auto name = path.filename().string();
    if (!name.empty() && name.front() == '.') {
      continue;
    }
    if (path_has_pcd_extension(path)) {
      files.push_back(path.string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

}  // namespace clc
