#include <clc_core/scan_geometry.hpp>

#include <clc_core/pcd_io.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

namespace clc {

bool read_pcd_dimensions(const std::string& path, int& width, int& height, std::string& error) {
  pcl::PCLPointCloud2 header;
  pcl::PCDReader reader;
  if (reader.readHeader(path, header) != 0) {
    error = "failed to read PCD header: " + path;
    return false;
  }
  width = static_cast<int>(header.width);
  height = static_cast<int>(header.height);
  if (width <= 0 || height <= 0) {
    error = "PCD has invalid dimensions: " + path;
    return false;
  }
  return true;
}

PcdDirectoryScan scan_pcd_directory(const std::string& directory, int reference_width, int reference_height) {
  PcdDirectoryScan result;
  const auto candidates = list_pcd_files(directory);
  if (candidates.empty()) {
    result.error = "no .pcd files in directory: " + directory;
    return result;
  }

  std::vector<std::string> readable;
  for (const auto& path : candidates) {
    int w = 0;
    int h = 0;
    std::string err;
    if (read_pcd_dimensions(path, w, h, err)) {
      readable.push_back(path);
    }
  }
  if (readable.empty()) {
    result.error = "no readable structured .pcd files in directory: " + directory;
    return result;
  }

  int ref_w = reference_width;
  int ref_h = reference_height;

  for (const auto& path : readable) {
    int w = 0;
    int h = 0;
    std::string err;
    if (!read_pcd_dimensions(path, w, h, err)) {
      continue;
    }
    if (ref_w <= 0 || ref_h <= 0) {
      ref_w = w;
      ref_h = h;
      continue;
    }
    if (w != ref_w || h != ref_h) {
      result.error = "scan dimension mismatch in " + path + " (" + std::to_string(w) + "x" + std::to_string(h) +
                     " vs reference " + std::to_string(ref_w) + "x" + std::to_string(ref_h) + ")";
      result.paths.clear();
      return result;
    }
  }

  result.paths = std::move(readable);
  result.reference_width = ref_w;
  result.reference_height = ref_h;
  return result;
}

}  // namespace clc
