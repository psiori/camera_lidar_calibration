#pragma once

#include <vector>

#include <clc/linalg.hpp>
#include <camera/generic_camera_base.hpp>
#include <opencv2/core.hpp>

namespace clc {

/// Qt-free camera–lidar color projection (vlcal PointsColorUpdater logic).
class PointColorModel {
 public:
  void setCamera(camera::GenericCameraBase::ConstPtr camera);
  void setImage(const cv::Mat& image_bgr);
  void setPoints(const std::vector<Vec3d>& points, const std::vector<double>& intensities);

  /// Per-point camera RGB in [0, 1] (black when not visible in the image). Blend with intensity colormap on the GPU.
  void projectCameraRgb(const Mat4d& T_camera_lidar, std::vector<float>& rgb_out) const;

  bool ready() const { return camera_ && !image_.empty() && !points_.empty(); }
  size_t pointCount() const { return points_.size(); }

 private:
  camera::GenericCameraBase::ConstPtr camera_;
  cv::Mat image_;
  std::vector<Vec3d> points_;
  std::vector<double> intensities_;
  double min_nz_ = 0.0;
};

}  // namespace clc
