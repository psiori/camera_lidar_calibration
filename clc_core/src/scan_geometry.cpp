#include <clc_core/scan_geometry.hpp>

#include <clc_core/pcd_io.hpp>

#include <cctype>
#include <filesystem>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

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

PcdDirectoryScan scan_pcd_source(const std::string& path, int reference_width, int reference_height) {
  PcdDirectoryScan result;
  if (path.empty()) {
    result.error = "PCD path is empty";
    return result;
  }
  std::error_code ec;
  const std::filesystem::path fs_path(path);
  if (std::filesystem::is_regular_file(fs_path, ec)) {
    if (!path_has_pcd_extension(fs_path)) {
      result.error = "not a PCD file: " + path;
      return result;
    }
    int w = 0;
    int h = 0;
    std::string err;
    if (!read_pcd_dimensions(path, w, h, err)) {
      result.error = err;
      return result;
    }
    result.paths = {path};
    result.reference_width = w;
    result.reference_height = h;
    return result;
  }
  if (std::filesystem::is_directory(fs_path, ec)) {
    return scan_pcd_directory(path, reference_width, reference_height);
  }
  if (path_has_pcd_extension(fs_path)) {
    result.error = "PCD file not found: " + path;
  } else {
    result.error = "path is not a directory or .pcd file: " + path;
  }
  return result;
}

}  // namespace clc
