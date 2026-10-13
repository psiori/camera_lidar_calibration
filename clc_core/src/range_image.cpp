#include <clc_core/range_image.hpp>

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace clc {

RangeImage build_range_image(const PointXYZITCloud& cloud) {
  RangeImage image;
  image.width = cloud.width;
  image.height = cloud.height;
  const std::size_t n = cloud.scan.points.size();
  image.range.assign(n, 0.f);
  image.intensity.assign(n, 0.f);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& p = cloud.scan.points[i];
    if (p.array().isFinite().all()) {
      image.range[i] = static_cast<float>(std::sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z()));
    }
    if (i < cloud.scan.intensities.size()) {
      image.intensity[i] = static_cast<float>(cloud.scan.intensities[i]);
    }
  }
  return image;
}

std::vector<std::uint8_t> rasterize_mask(int width, int height, const std::vector<Polygon2f>& polygons, bool keep_inside) {
  if (width <= 0 || height <= 0) {
    return {};
  }
  cv::Mat mask(height, width, CV_8UC1, keep_inside ? cv::Scalar(0) : cv::Scalar(255));
  std::vector<std::vector<cv::Point>> cv_polys;
  cv_polys.reserve(polygons.size());
  for (const auto& poly : polygons) {
    if (poly.vertices.size() < 3) {
      continue;
    }
    std::vector<cv::Point> pts;
    pts.reserve(poly.vertices.size());
    for (const auto& v : poly.vertices) {
      pts.emplace_back(static_cast<int>(std::lround(v.x())), static_cast<int>(std::lround(v.y())));
    }
    cv_polys.push_back(std::move(pts));
  }
  if (!cv_polys.empty()) {
    cv::fillPoly(mask, cv_polys, keep_inside ? cv::Scalar(255) : cv::Scalar(0));
  }
  return {mask.datastart, mask.dataend};
}

std::vector<std::uint8_t> build_mask_from_ops(int width, int height, const std::vector<MaskPolygonOp>& ops) {
  if (width <= 0 || height <= 0) {
    return {};
  }
  std::vector<std::uint8_t> mask(static_cast<std::size_t>(width * height), 255);
  if (ops.empty()) {
    return mask;
  }
  cv::Mat view(height, width, CV_8UC1, mask.data());
  for (const auto& op : ops) {
    if (op.polygon.vertices.size() < 3) {
      continue;
    }
    std::vector<cv::Point> pts;
    pts.reserve(op.polygon.vertices.size());
    for (const auto& v : op.polygon.vertices) {
      pts.emplace_back(static_cast<int>(std::lround(v.x())), static_cast<int>(std::lround(v.y())));
    }
    const std::vector<std::vector<cv::Point>> cv_polys{pts};
    cv::fillPoly(view, cv_polys, op.keep_inside ? cv::Scalar(255) : cv::Scalar(0));
  }
  return mask;
}

bool save_mask_png(const std::string& path, int width, int height, const std::vector<std::uint8_t>& mask) {
  if (width <= 0 || height <= 0 || static_cast<int>(mask.size()) != width * height) {
    return false;
  }
  const cv::Mat img(height, width, CV_8UC1, const_cast<std::uint8_t*>(mask.data()));
  return cv::imwrite(path, img);
}

bool load_mask_png(const std::string& path, int width, int height, std::vector<std::uint8_t>& mask, std::string& error) {
  const cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    error = "failed to load mask: " + path;
    return false;
  }
  if (img.cols != width || img.rows != height) {
    error = "mask size mismatch: got " + std::to_string(img.cols) + "x" + std::to_string(img.rows) + " expected " +
            std::to_string(width) + "x" + std::to_string(height);
    return false;
  }
  mask.assign(img.datastart, img.dataend);
  return true;
}

glim_cloud_fusion::LidarScan to_lidar_scan(const PointXYZITCloud& cloud, double stamp) {
  glim_cloud_fusion::LidarScan scan;
  scan.stamp = stamp;
  scan.width = cloud.width;
  scan.height = cloud.height;
  scan.points = cloud.scan.points;
  scan.times = cloud.scan.times;
  scan.intensities = cloud.scan.intensities;
  scan.keep = cloud.scan.keep;
  return scan;
}

glim_cloud_fusion::LidarScan to_lidar_scan_masked(
  const PointXYZITCloud& cloud,
  const std::vector<std::uint8_t>& mask_pixels,
  double stamp) {
  auto scan = to_lidar_scan(cloud, stamp);
  const std::size_t expected =
    static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (!mask_pixels.empty() && mask_pixels.size() == expected) {
    apply_mask_to_scan(scan, mask_pixels);
  }
  return scan;
}

void apply_mask_to_scan(glim_cloud_fusion::LidarScan& scan, const std::vector<std::uint8_t>& mask) {
  if (mask.empty()) {
    return;
  }
  const std::size_t n = scan.points.size();
  if (scan.keep.size() != n) {
    scan.keep.assign(n, true);
  }
  const std::size_t m = std::min(n, mask.size());
  for (std::size_t i = 0; i < m; ++i) {
    if (mask[i] == 0) {
      scan.keep[i] = false;
    }
  }
}

}  // namespace clc
