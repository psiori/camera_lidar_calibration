#include <clc_core/point_color_model.hpp>

#include <cmath>

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

void PointColorModel::setPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points,
                                const std::vector<double>& intensities) {
  points_ = points;
  intensities_ = intensities;
}

void PointColorModel::projectCameraRgb(const Eigen::Isometry3d& T_camera_lidar, std::vector<float>& rgb_out) const {
  const size_t n = points_.size();
  rgb_out.resize(n * 3);
  if (!camera_ || image_.empty() || n == 0) {
    std::fill(rgb_out.begin(), rgb_out.end(), 0.f);
    return;
  }

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector4d pt_camera = T_camera_lidar * Eigen::Vector4d(points_[i].x(), points_[i].y(), points_[i].z(), 1.0);
    bool in_image = true;
    if (pt_camera.head<3>().normalized().z() < min_nz_) {
      in_image = false;
    }
    const Eigen::Vector2i pt_2d = camera_->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() ||
        (pt_2d.array() >= Eigen::Array2i(image_.cols, image_.rows)).any()) {
      in_image = false;
    }

    if (!in_image) {
      rgb_out[i * 3 + 0] = 0.f;
      rgb_out[i * 3 + 1] = 0.f;
      rgb_out[i * 3 + 2] = 0.f;
      continue;
    }
    const cv::Vec3b pix = image_.at<cv::Vec3b>(pt_2d.y(), pt_2d.x());
    rgb_out[i * 3 + 0] = pix[2] / 255.f;
    rgb_out[i * 3 + 1] = pix[1] / 255.f;
    rgb_out[i * 3 + 2] = pix[0] / 255.f;
  }
}

}  // namespace clc
