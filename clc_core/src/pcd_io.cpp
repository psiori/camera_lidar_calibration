#include <clc_core/pcd_io.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>

#define PCL_NO_PRECOMPILE
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

namespace clc {

bool parse_pcd_filename_stamp(const std::string& filename, double& stamp_seconds) {
  int y = 0;
  unsigned m = 0, d = 0, h = 0, mi = 0, s = 0, ms = 0;
  if (std::sscanf(filename.c_str(), "%d-%u-%uT%u_%u_%u_%3uZ.pcd", &y, &m, &d, &h, &mi, &s, &ms) != 7) {
    return false;
  }
  if (m < 1 || m > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || s > 59 || ms > 999) {
    return false;
  }
  std::tm calendar_time{};
  calendar_time.tm_sec = static_cast<int>(s);
  calendar_time.tm_min = static_cast<int>(mi);
  calendar_time.tm_hour = static_cast<int>(h);
  calendar_time.tm_mday = static_cast<int>(d);
  calendar_time.tm_mon = static_cast<int>(m) - 1;
  calendar_time.tm_year = y - 1900;
  calendar_time.tm_isdst = 0;
#if defined(_WIN32)
  const std::time_t utc_time = _mkgmtime(&calendar_time);
#else
  const std::time_t utc_time = timegm(&calendar_time);
#endif
  if (utc_time == static_cast<std::time_t>(-1)) {
    return false;
  }
  const auto ms_tp =
    std::chrono::milliseconds(ms) + std::chrono::system_clock::from_time_t(utc_time);
  stamp_seconds =
    std::chrono::duration_cast<std::chrono::duration<double>>(ms_tp.time_since_epoch()).count();
  return true;
}

namespace {

bool path_has_pcd_extension(const std::filesystem::path& path) {
  std::string ext = path.extension().string();
  for (char& c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return ext == ".pcd";
}

const pcl::PCLPointField* field_by_index(const pcl::PCLPointCloud2& cloud, int index) {
  if (index < 0 || static_cast<std::size_t>(index) >= cloud.fields.size()) {
    return nullptr;
  }
  return &cloud.fields[static_cast<std::size_t>(index)];
}

bool read_field_as_float(const std::uint8_t* point_data, const pcl::PCLPointField& field, float& out) {
  switch (field.datatype) {
    case pcl::PCLPointField::FLOAT32:
      std::memcpy(&out, point_data + field.offset, sizeof(float));
      return true;
    case pcl::PCLPointField::FLOAT64: {
      double v = 0.0;
      std::memcpy(&v, point_data + field.offset, sizeof(double));
      out = static_cast<float>(v);
      return true;
    }
    default:
      return false;
  }
}

bool read_field_as_double(const std::uint8_t* point_data, const pcl::PCLPointField& field, double& out) {
  switch (field.datatype) {
    case pcl::PCLPointField::FLOAT32: {
      float v = 0.f;
      std::memcpy(&v, point_data + field.offset, sizeof(float));
      out = static_cast<double>(v);
      return true;
    }
    case pcl::PCLPointField::FLOAT64:
      std::memcpy(&out, point_data + field.offset, sizeof(double));
      return true;
    case pcl::PCLPointField::UINT64: {
      std::uint64_t v = 0;
      std::memcpy(&v, point_data + field.offset, sizeof(std::uint64_t));
      out = static_cast<double>(v);
      return true;
    }
    case pcl::PCLPointField::INT64: {
      std::int64_t v = 0;
      std::memcpy(&v, point_data + field.offset, sizeof(std::int64_t));
      out = static_cast<double>(v);
      return true;
    }
    case pcl::PCLPointField::UINT32: {
      std::uint32_t v = 0;
      std::memcpy(&v, point_data + field.offset, sizeof(std::uint32_t));
      out = static_cast<double>(v);
      return true;
    }
    case pcl::PCLPointField::INT32: {
      std::int32_t v = 0;
      std::memcpy(&v, point_data + field.offset, sizeof(std::int32_t));
      out = static_cast<double>(v);
      return true;
    }
    default:
      return false;
  }
}

std::uint8_t float_intensity_unit_to_uint8(float value) {
  const int scaled = static_cast<int>(std::lround(value * 255.f));
  return static_cast<std::uint8_t>(std::clamp(scaled, 0, 255));
}

std::uint8_t float_intensity_byte_to_uint8(float value) {
  const int scaled = static_cast<int>(std::lround(value));
  return static_cast<std::uint8_t>(std::clamp(scaled, 0, 255));
}

enum class IntensityDecode { None, Uint8, Float };

enum class FloatIntensityRange { Unit01, Byte0255 };

struct IntensityField {
  IntensityDecode decode = IntensityDecode::None;
  FloatIntensityRange float_range = FloatIntensityRange::Unit01;
  const pcl::PCLPointField* field = nullptr;
};

constexpr std::size_t k_intensity_range_probe_stride = 100;

bool float_intensity_sampled_values_in_unit_range(const pcl::PCLPointCloud2& cloud,
                                                  const pcl::PCLPointField& field,
                                                  std::size_t num_points) {
  for (std::size_t i = 0; i < num_points; i += k_intensity_range_probe_stride) {
    const std::uint8_t* point_data = cloud.data.data() + i * cloud.point_step;
    float value = 0.f;
    if (!read_field_as_float(point_data, field, value) || !std::isfinite(value)) {
      return false;
    }
    if (value < 0.f || value > 1.f) {
      return false;
    }
  }
  return true;
}

void resolve_float_intensity_range(const pcl::PCLPointCloud2& cloud,
                                   IntensityField& intensity,
                                   std::size_t num_points) {
  if (intensity.decode != IntensityDecode::Float || intensity.field == nullptr) {
    return;
  }
  intensity.float_range = float_intensity_sampled_values_in_unit_range(cloud, *intensity.field, num_points)
                            ? FloatIntensityRange::Unit01
                            : FloatIntensityRange::Byte0255;
}

IntensityField resolve_intensity_field(const pcl::PCLPointCloud2& cloud, std::string& error) {
  IntensityField result;
  const int intensity_idx = pcl::getFieldIndex(cloud, "intensity");
  const int reflectivity_idx = pcl::getFieldIndex(cloud, "reflectivity");
  const int field_idx = intensity_idx >= 0 ? intensity_idx : reflectivity_idx;
  if (field_idx < 0) {
    return result;
  }
  result.field = field_by_index(cloud, field_idx);
  if (result.field == nullptr) {
    error = "PCD intensity field index is invalid";
    return result;
  }
  switch (result.field->datatype) {
    case pcl::PCLPointField::UINT8:
      result.decode = IntensityDecode::Uint8;
      break;
    case pcl::PCLPointField::FLOAT32:
    case pcl::PCLPointField::FLOAT64:
      result.decode = IntensityDecode::Float;
      break;
    default:
      error = "unsupported intensity field type in PCD (expected uint8 or float)";
      result.field = nullptr;
      break;
  }
  return result;
}

std::uint8_t read_intensity_uint8(const std::uint8_t* point_data, const IntensityField& intensity) {
  if (intensity.decode == IntensityDecode::None || intensity.field == nullptr) {
    return 0;
  }
  switch (intensity.decode) {
    case IntensityDecode::Uint8: {
      std::uint8_t value = 0;
      std::memcpy(&value, point_data + intensity.field->offset, sizeof(std::uint8_t));
      return value;
    }
    case IntensityDecode::Float: {
      float value = 0.f;
      if (!read_field_as_float(point_data, *intensity.field, value)) {
        return 0;
      }
      if (intensity.float_range == FloatIntensityRange::Unit01) {
        return float_intensity_unit_to_uint8(value);
      }
      return float_intensity_byte_to_uint8(value);
    }
    default:
      return 0;
  }
}

const pcl::PCLPointField* resolve_time_field(const pcl::PCLPointCloud2& cloud) {
  for (const char* name : {"time", "timestamp", "t"}) {
    const int idx = pcl::getFieldIndex(cloud, name);
    if (idx >= 0) {
      return field_by_index(cloud, idx);
    }
  }
  return nullptr;
}

}  // namespace

bool load_dense_pcd(const std::string& path, glim_cloud_fusion::DensePointCloud& cloud, std::string& error) {
  if (!path_has_pcd_extension(std::filesystem::path(path))) {
    error = "not a PCD file: " + path;
    return false;
  }

  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(path, pcl_cloud) != 0) {
    error = "failed to load PCD: " + path;
    return false;
  }

  const int x_idx = pcl::getFieldIndex(pcl_cloud, "x");
  const int y_idx = pcl::getFieldIndex(pcl_cloud, "y");
  const int z_idx = pcl::getFieldIndex(pcl_cloud, "z");
  const auto* x_field = field_by_index(pcl_cloud, x_idx);
  const auto* y_field = field_by_index(pcl_cloud, y_idx);
  const auto* z_field = field_by_index(pcl_cloud, z_idx);
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    error = "PCD is missing x/y/z fields: " + path;
    return false;
  }

  std::string intensity_error;
  IntensityField intensity = resolve_intensity_field(pcl_cloud, intensity_error);
  if (!intensity_error.empty()) {
    error = intensity_error;
    return false;
  }

  const std::size_t num_points =
    static_cast<std::size_t>(pcl_cloud.width) * static_cast<std::size_t>(pcl_cloud.height);
  if (num_points == 0 || pcl_cloud.point_step == 0) {
    error = "PCD contains no points: " + path;
    return false;
  }

  cloud.points.clear();
  cloud.intensities.clear();
  cloud.points.reserve(num_points);
  cloud.intensities.reserve(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    const std::uint8_t* point_data = pcl_cloud.data.data() + i * pcl_cloud.point_step;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    if (!read_field_as_float(point_data, *x_field, x) || !read_field_as_float(point_data, *y_field, y) ||
        !read_field_as_float(point_data, *z_field, z)) {
      error = "PCD x/y/z fields are not floating point: " + path;
      return false;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    cloud.points.emplace_back(x, y, z);
    double intensity_value = 0.0;
    if (intensity.decode == IntensityDecode::Float && intensity.field != nullptr) {
      read_field_as_double(point_data, *intensity.field, intensity_value);
    } else if (intensity.decode == IntensityDecode::Uint8 && intensity.field != nullptr) {
      intensity_value = static_cast<double>(read_intensity_uint8(point_data, intensity));
    }
    cloud.intensities.push_back(intensity_value);
  }

  if (cloud.points.empty()) {
    error = "PCD contains no finite points: " + path;
    return false;
  }
  return true;
}

bool load_pcd_xyzit(const std::string& path, PointXYZITCloud& cloud, std::string& error) {
  if (!path_has_pcd_extension(path)) {
    error = "not a PCD file: " + path;
    return false;
  }

  const auto basename = std::filesystem::path(path).filename().string();
  double stamp_seconds = 0.0;
  if (!parse_pcd_filename_stamp(basename, stamp_seconds)) {
    error = "PCD filename does not match UTC timestamp pattern (YYYY-MM-DDTHH_MM_SS_mmmZ.pcd): " + path;
    return false;
  }

  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(path, pcl_cloud) != 0) {
    error = "failed to load PCD: " + path;
    return false;
  }

  const int x_idx = pcl::getFieldIndex(pcl_cloud, "x");
  const int y_idx = pcl::getFieldIndex(pcl_cloud, "y");
  const int z_idx = pcl::getFieldIndex(pcl_cloud, "z");
  const auto* x_field = field_by_index(pcl_cloud, x_idx);
  const auto* y_field = field_by_index(pcl_cloud, y_idx);
  const auto* z_field = field_by_index(pcl_cloud, z_idx);
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    error = "PCD is missing x/y/z fields: " + path;
    return false;
  }

  IntensityField intensity = resolve_intensity_field(pcl_cloud, error);
  if (!error.empty()) {
    return false;
  }
  const pcl::PCLPointField* time_field = resolve_time_field(pcl_cloud);

  const std::size_t num_points =
    static_cast<std::size_t>(pcl_cloud.width) * static_cast<std::size_t>(pcl_cloud.height);
  if (num_points == 0 || pcl_cloud.point_step == 0) {
    error = "PCD contains no points";
    return false;
  }

  resolve_float_intensity_range(pcl_cloud, intensity, num_points);

  cloud.width = static_cast<int>(pcl_cloud.width);
  cloud.height = static_cast<int>(pcl_cloud.height);
  cloud.scan.points.clear();
  cloud.scan.times.clear();
  cloud.scan.intensities.clear();
  cloud.scan.keep.clear();
  cloud.scan.width = cloud.width;
  cloud.scan.height = cloud.height;
  cloud.scan.points.reserve(num_points);

  bool any_valid = false;
  for (std::size_t i = 0; i < num_points; ++i) {
    const std::uint8_t* point_data = pcl_cloud.data.data() + i * pcl_cloud.point_step;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    if (!read_field_as_float(point_data, *x_field, x) || !read_field_as_float(point_data, *y_field, y) ||
        !read_field_as_float(point_data, *z_field, z)) {
      error = "PCD x/y/z fields are not floating point: " + path;
      return false;
    }

    double time = 0.0;
    if (time_field != nullptr && !read_field_as_double(point_data, *time_field, time)) {
      error = "PCD time field has unsupported type: " + path;
      return false;
    }

    const bool valid = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    const std::uint8_t intensity_u8 = read_intensity_uint8(point_data, intensity);
    cloud.scan.points.emplace_back(x, y, z, 1.0);
    cloud.scan.times.push_back(time);
    cloud.scan.intensities.push_back(static_cast<double>(intensity_u8));
    cloud.scan.keep.push_back(valid);
    any_valid = any_valid || valid;
  }

  if (!any_valid) {
    error = "PCD contains no valid points";
    return false;
  }

  cloud.stamp = stamp_seconds;
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
    if (!path_has_pcd_extension(path)) {
      continue;
    }
    double stamp_seconds = 0.0;
    if (!parse_pcd_filename_stamp(name, stamp_seconds)) {
      continue;
    }
    files.push_back(path.string());
  }
  std::sort(files.begin(), files.end());
  return files;
}

}  // namespace clc
