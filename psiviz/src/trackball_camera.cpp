#include "psiviz/trackball_camera.hpp"

#include <algorithm>
#include <cmath>

namespace psiviz {

namespace {

constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kDefaultPitch = 0.3f;

Eigen::Vector3f rotateAroundAxis(const Eigen::Vector3f& point, const Eigen::Vector3f& center,
                                 const Eigen::Vector3f& axis_unit, float angle_rad) {
  const Eigen::AngleAxisf rot(angle_rad, axis_unit);
  return center + rot * (point - center);
}

Eigen::Vector3f defaultForwardFromPitchYaw(float pitch, float yaw) {
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  Eigen::Vector3f forward(cp * cy, sp, cp * sy);
  const float n = forward.norm();
  if (n > 1e-8f) {
    forward /= n;
  }
  return forward;
}

}  // namespace

TrackballCamera::TrackballCamera() {
  view_up_ = Eigen::Vector3f(0.f, 1.f, 0.f);
  setOrbitFromForward(10.f);
}

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

float TrackballCamera::orbitDistance() const {
  return (position_ - focal_).norm();
}

void TrackballCamera::setOrbitFromForward(float orbit_distance) {
  const float d = std::max(0.05f, orbit_distance);
  const Eigen::Vector3f forward = defaultForwardFromPitchYaw(kDefaultPitch, 0.f);
  position_ = focal_ - forward * d;
  view_up_ = Eigen::Vector3f(0.f, 1.f, 0.f);
  orthogonalizeViewUp();
}

void TrackballCamera::orthogonalizeViewUp() {
  Eigen::Vector3f forward = focal_ - position_;
  if (forward.squaredNorm() < 1e-12f) {
    return;
  }
  forward.normalize();
  view_up_ -= forward * forward.dot(view_up_);
  if (view_up_.squaredNorm() < 1e-12f) {
    view_up_ = Eigen::Vector3f(0.f, 1.f, 0.f);
    view_up_ -= forward * forward.dot(view_up_);
  }
  if (view_up_.squaredNorm() > 1e-12f) {
    view_up_.normalize();
  }
}

void TrackballCamera::fitToBounds(const Eigen::Vector3f& min, const Eigen::Vector3f& max) {
  const Eigen::Vector3f center = (min + max) * 0.5f;
  const float radius = (max - min).norm() * 0.5f;
  focal_ = center;
  const float distance = std::max(1.f, radius * 2.5f);
  z_far_ = std::max(1000.f, distance * 8.f);
  z_near_ = std::max(0.001f, distance * 1e-4f);
  point_size_ = std::max(point_size_, 3.f);
  setOrbitFromForward(distance);
}

void TrackballCamera::rotate(float dx, float dy) {
  const float az_deg = dx * (-20.f / static_cast<float>(width_)) * motion_factor_;
  // Positive screen Y is down; drag up (dy < 0) should pitch view down (PCL/VTK feel).
  const float el_deg = dy * (20.f / static_cast<float>(height_)) * motion_factor_;

  Eigen::Vector3f up = view_up_;
  if (up.squaredNorm() < 1e-12f) {
    up = Eigen::Vector3f(0.f, 1.f, 0.f);
  }
  up.normalize();

  if (std::abs(az_deg) > 1e-6f) {
    position_ = rotateAroundAxis(position_, focal_, up, az_deg * kDegToRad);
  }

  Eigen::Vector3f forward = focal_ - position_;
  if (forward.squaredNorm() < 1e-12f) {
    return;
  }
  forward.normalize();

  Eigen::Vector3f elevation_axis = view_up_.cross(forward);
  if (elevation_axis.squaredNorm() < 1e-12f) {
    orthogonalizeViewUp();
    return;
  }
  elevation_axis.normalize();

  if (std::abs(el_deg) > 1e-6f) {
    position_ = rotateAroundAxis(position_, focal_, elevation_axis, el_deg * kDegToRad);
  }

  orthogonalizeViewUp();
}

void TrackballCamera::pan(float dx, float dy) {
  const Eigen::Matrix4f view = viewMatrix();
  const Eigen::Vector3f right = view.block<1, 3>(0, 0).transpose();
  const Eigen::Vector3f up = view.block<1, 3>(1, 0).transpose();
  const float scale = orbitDistance() * 0.001f;
  const Eigen::Vector3f delta = -right * dx * scale + up * dy * scale;
  focal_ += delta;
  position_ += delta;
}

void TrackballCamera::dolly(float delta) {
  Eigen::Vector3f forward = focal_ - position_;
  const float dist = forward.norm();
  if (dist < 1e-8f) {
    return;
  }
  forward /= dist;
  float new_dist = dist * std::exp(-delta * 0.001f);
  new_dist = std::clamp(new_dist, 0.05f, 1e6f);
  position_ = focal_ - forward * new_dist;
}

void TrackballCamera::spin(float dx, float dy) {
  Eigen::Vector3f forward = focal_ - position_;
  if (forward.squaredNorm() < 1e-12f) {
    return;
  }
  forward.normalize();
  const float angle = (dx + dy) * 0.003f;
  view_up_ = Eigen::AngleAxisf(angle, forward) * view_up_;
  orthogonalizeViewUp();
}

void TrackballCamera::reset() {
  setOrbitFromForward(orbitDistance());
}

void TrackballCamera::flyTo(const Eigen::Vector3f& target, float duration_sec) {
  fly_target_ = target;
  fly_start_focal_ = focal_;
  fly_start_position_ = position_;
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
  position_ = fly_start_position_ + (fly_target_ - fly_start_focal_) * s;
}

Eigen::Matrix4f TrackballCamera::viewMatrix() const {
  const Eigen::Vector3f eye = position_;
  const Eigen::Vector3f center = focal_;
  Eigen::Vector3f up = view_up_;

  Eigen::Vector3f f = center - eye;
  if (f.squaredNorm() < 1e-12f) {
    f = Eigen::Vector3f(0.f, 0.f, -1.f);
  } else {
    f.normalize();
  }

  Eigen::Vector3f s = f.cross(up);
  if (s.squaredNorm() < 1e-12f) {
    up = Eigen::Vector3f(0.f, 1.f, 0.f);
    s = f.cross(up);
  }
  s.normalize();
  const Eigen::Vector3f u = s.cross(f);

  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view.block<1, 3>(0, 0) = s.transpose();
  view.block<1, 3>(1, 0) = u.transpose();
  view.block<1, 3>(2, 0) = -f.transpose();
  view(0, 3) = -s.dot(eye);
  view(1, 3) = -u.dot(eye);
  view(2, 3) = f.dot(eye);
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
