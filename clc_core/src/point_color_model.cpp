#include <clc_core/point_color_model.hpp>

#include <cmath>

#include <clc_core/colormap.hpp>
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

void PointColorModel::projectColors(const Eigen::Isometry3d& T_camera_lidar, float blend_weight, float scalar_min,
                                    float scalar_max, std::vector<float>& rgb_out) const {
  const size_t n = points_.size();
  rgb_out.resize(n * 3);
  if (!camera_ || image_.empty() || n == 0) {
    return;
  }

  const float blend = std::clamp(blend_weight, 0.f, 1.f);
  const float inv_blend = 1.f - blend;
  const float span = scalar_max > scalar_min ? (scalar_max - scalar_min) : 1.f;

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

    float cr = 0.5f;
    float cg = 0.5f;
    float cb = 0.5f;
    if (in_image) {
      const cv::Vec3b pix = image_.at<cv::Vec3b>(pt_2d.y(), pt_2d.x());
      cb = pix[0] / 255.f;
      cg = pix[1] / 255.f;
      cr = pix[2] / 255.f;
    }

    const double intensity = i < intensities_.size() ? intensities_[i] : 0.0;
    const float t = std::clamp(static_cast<float>((intensity - scalar_min) / span), 0.f, 1.f);
    const auto jet = jet_color(t);

    rgb_out[i * 3 + 0] = cr * blend + jet[0] * inv_blend;
    rgb_out[i * 3 + 1] = cg * blend + jet[1] * inv_blend;
    rgb_out[i * 3 + 2] = cb * blend + jet[2] * inv_blend;
  }
}

}  // namespace clc
