// point_color_model.cpp — clc_core session, I/O, fusion, or calibration logic.

#include <clc_core/point_color_model.hpp>

#include <cmath>

#include "eigen_adapters.hpp"
#include <vlcal/common/estimate_fov.hpp>

namespace clc {

void PointColorModel::setCamera(camera::GenericCameraBase::ConstPtr camera) {
  camera_ = std::move(camera);
  min_nz_ = 0.0;
}

void PointColorModel::setImage(const cv::Mat& image_bgr) {
  image_ = image_bgr;
  if (camera_ && !image_.empty()) {
    min_nz_ = std::cos(vlcal::estimate_camera_fov(camera_, {image_.cols, image_.rows}) + 0.5 * M_PI / 180.0);
  }
}

void PointColorModel::setPoints(const std::vector<Vec3d>& points, const std::vector<double>& intensities) {
  points_ = points;
  intensities_ = intensities;
}

void PointColorModel::projectCameraRgb(const Mat4d& T_camera_lidar, std::vector<float>& rgb_out) const {
  const size_t n = points_.size();
  rgb_out.resize(n * 3);
  if (!camera_ || image_.empty() || n == 0) {
    std::fill(rgb_out.begin(), rgb_out.end(), 0.f);
    return;
  }

  for (size_t i = 0; i < n; ++i) {
    const Vec3d pt_cam = transformPoint(T_camera_lidar, points_[i]);
    const Eigen::Vector3d pt_cam_e = detail::toEigen(pt_cam);
    bool in_image = true;
    const double norm = std::sqrt(pt_cam_e.squaredNorm());
    if (norm > 1e-12 && pt_cam_e.z() / norm < min_nz_) {
      in_image = false;
    }
    const Eigen::Vector2d pt_2d = camera_->project(pt_cam_e);
    if (pt_2d.x() < 0. || pt_2d.y() < 0. || pt_2d.x() >= image_.cols || pt_2d.y() >= image_.rows) {
      in_image = false;
    }

    if (!in_image) {
      rgb_out[i * 3 + 0] = 0.f;
      rgb_out[i * 3 + 1] = 0.f;
      rgb_out[i * 3 + 2] = 0.f;
      continue;
    }
    const cv::Vec3b pix = image_.at<cv::Vec3b>(static_cast<int>(pt_2d.y()), static_cast<int>(pt_2d.x()));
    rgb_out[i * 3 + 0] = pix[2] / 255.f;
    rgb_out[i * 3 + 1] = pix[1] / 255.f;
    rgb_out[i * 3 + 2] = pix[0] / 255.f;
  }
}

}  // namespace clc
