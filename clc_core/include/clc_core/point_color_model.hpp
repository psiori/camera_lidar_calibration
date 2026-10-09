#pragma once

#include <vector>

#include <camera/generic_camera_base.hpp>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace clc {

/// Qt-free camera–lidar color projection (vlcal PointsColorUpdater logic).
class PointColorModel {
 public:
  void setCamera(camera::GenericCameraBase::ConstPtr camera);
  void setImage(const cv::Mat& image_bgr);
  void setPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points,
                 const std::vector<double>& intensities);

  /// Writes interleaved RGB in [0, 1]; size must be points.size() * 3.
  void projectColors(const Eigen::Isometry3d& T_camera_lidar, float blend_weight, float scalar_min, float scalar_max,
                     std::vector<float>& rgb_out) const;

  bool ready() const { return camera_ && !image_.empty() && !points_.empty(); }
  size_t pointCount() const { return points_.size(); }

 private:
  camera::GenericCameraBase::ConstPtr camera_;
  cv::Mat image_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points_;
  std::vector<double> intensities_;
  double min_nz_ = 0.0;
};

}  // namespace clc
