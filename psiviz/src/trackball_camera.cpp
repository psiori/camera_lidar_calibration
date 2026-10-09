#include "psiviz/trackball_camera.hpp"

#include <algorithm>
#include <cmath>

namespace psiviz {

namespace {
constexpr float kDegToRad = 0.017453292519943295f;
}

TrackballCamera::TrackballCamera() = default;

void TrackballCamera::setViewport(int width, int height, float device_pixel_ratio) {
  width_ = std::max(1, width);
  height_ = std::max(1, height);
  dpr_ = std::max(0.5f, device_pixel_ratio);
}

void TrackballCamera::setPerspective(float fovy_deg, float z_near, float z_far) {
  fovy_deg_ = fovy_deg;
  z_near_ = z_near;
  z_far_ = z_far;
}

void TrackballCamera::fitToBounds(const Eigen::Vector3f& min, const Eigen::Vector3f& max) {
  const Eigen::Vector3f center = (min + max) * 0.5f;
  const float radius = (max - min).norm() * 0.5f;
  focal_ = center;
  distance_ = std::max(1.f, radius * 2.5f);
  z_far_ = std::max(1000.f, distance_ * 8.f);
  z_near_ = std::max(0.001f, distance_ * 1e-4f);
  point_size_ = std::max(point_size_, 3.f);
  yaw_ = 0.f;
  pitch_ = 0.3f;
  roll_ = 0.f;
}

void TrackballCamera::rotate(float dx, float dy) {
  yaw_ += dx * 0.005f;
  pitch_ += dy * 0.005f;
  pitch_ = std::clamp(pitch_, -1.4f, 1.4f);
}

void TrackballCamera::pan(float dx, float dy) {
  const Eigen::Matrix4f view = viewMatrix();
  const Eigen::Vector3f right = view.block<3, 1>(0, 0);
  const Eigen::Vector3f up = view.block<3, 1>(0, 1);
  const float scale = distance_ * 0.001f;
  focal_ -= right * dx * scale;
  focal_ += up * dy * scale;
}

void TrackballCamera::dolly(float delta) {
  distance_ *= std::exp(-delta * 0.001f);
  distance_ = std::clamp(distance_, 0.05f, 1e6f);
}

void TrackballCamera::spin(float dx, float dy) {
  roll_ += (dx + dy) * 0.003f;
}

void TrackballCamera::reset() {
  pitch_ = 0.3f;
  yaw_ = 0.f;
  roll_ = 0.f;
}

void TrackballCamera::flyTo(const Eigen::Vector3f& target, float duration_sec) {
  fly_target_ = target;
  fly_start_focal_ = focal_;
  fly_duration_ = std::max(0.05f, duration_sec);
  fly_t_ = 0.f;
}

void TrackballCamera::tick(float dt_sec) {
  if (fly_t_ >= 1.f) {
    return;
  }
  fly_t_ = std::min(1.f, fly_t_ + dt_sec / fly_duration_);
  const float s = fly_t_ * fly_t_ * (3.f - 2.f * fly_t_);
  focal_ = fly_start_focal_ + (fly_target_ - fly_start_focal_) * s;
}

Eigen::Matrix4f TrackballCamera::viewMatrix() const {
  const float cy = std::cos(yaw_);
  const float sy = std::sin(yaw_);
  const float cp = std::cos(pitch_);
  const float sp = std::sin(pitch_);
  const float cr = std::cos(roll_);
  const float sr = std::sin(roll_);

  Eigen::Vector3f forward(cp * cy, sp, cp * sy);
  Eigen::Vector3f eye = focal_ - forward.normalized() * distance_;

  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  Eigen::Vector3f up0(0.f, 1.f, 0.f);
  Eigen::Vector3f z = (focal_ - eye).normalized();
  Eigen::Vector3f x = z.cross(up0).normalized();
  Eigen::Vector3f y = z.cross(x);

  const Eigen::AngleAxisf roll_rot(roll_, z);
  x = roll_rot * x;
  y = roll_rot * y;

  view.block<1, 3>(0, 0) = x.transpose();
  view.block<1, 3>(1, 0) = y.transpose();
  view.block<1, 3>(2, 0) = z.transpose();
  view(0, 3) = -x.dot(eye);
  view(1, 3) = -y.dot(eye);
  view(2, 3) = -z.dot(eye);
  return view;
}

Eigen::Matrix4f TrackballCamera::projectionMatrix() const {
  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const float f = 1.f / std::tan(fovy_deg_ * kDegToRad * 0.5f);
  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
  proj(0, 0) = f / aspect;
  proj(1, 1) = f;
  proj(2, 2) = (z_far_ + z_near_) / (z_near_ - z_far_);
  proj(2, 3) = (2.f * z_far_ * z_near_) / (z_near_ - z_far_);
  proj(3, 2) = -1.f;
  return proj;
}

}  // namespace psiviz
