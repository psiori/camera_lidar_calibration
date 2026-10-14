// trackball_camera.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/trackball_camera.hpp"

#include <algorithm>
#include <cmath>

namespace psiviz {

namespace {

constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kDefaultPitch = 0.3f;

clc::Vec3f defaultForwardFromPitchYaw(float pitch, float yaw) {
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  clc::Vec3f forward{cp * cy, sp, cp * sy};
  const float n = clc::length(forward);
  if (n > 1e-8f) {
    forward = forward / n;
  }
  return forward;
}

}  // namespace

TrackballCamera::TrackballCamera() {
  view_up_ = {0.f, 1.f, 0.f};
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

float TrackballCamera::orbitDistance() const { return clc::length(position_ - focal_); }

void TrackballCamera::setOrbitFromForward(float orbit_distance) {
  const float d = std::max(0.05f, orbit_distance);
  const clc::Vec3f forward = defaultForwardFromPitchYaw(kDefaultPitch, 0.f);
  position_ = focal_ - forward * d;
  view_up_ = {0.f, 1.f, 0.f};
  orthogonalizeViewUp();
}

void TrackballCamera::orthogonalizeViewUp() {
  clc::Vec3f forward = focal_ - position_;
  if (clc::lengthSquared(forward) < 1e-12f) {
    return;
  }
  forward = clc::normalized(forward);
  view_up_ = view_up_ - forward * clc::dot(forward, view_up_);
  if (clc::lengthSquared(view_up_) < 1e-12f) {
    view_up_ = {0.f, 1.f, 0.f};
    view_up_ = view_up_ - forward * clc::dot(forward, view_up_);
  }
  if (clc::lengthSquared(view_up_) > 1e-12f) {
    view_up_ = clc::normalized(view_up_);
  }
}

void TrackballCamera::fitToBounds(const clc::Vec3f& min, const clc::Vec3f& max) {
  const clc::Vec3f center = (min + max) * 0.5f;
  const float radius = clc::length(max - min) * 0.5f;
  focal_ = center;
  const float distance = std::max(1.f, radius * 2.5f);
  z_far_ = std::max(100.f, distance * 10.f);
  z_near_ = std::max(0.001f, distance / 10.f);
  point_size_ = std::max(point_size_, 3.f);
  setOrbitFromForward(distance);
}

void TrackballCamera::rotate(float dx, float dy) {
  const float az_deg = dx * (-20.f / static_cast<float>(width_)) * motion_factor_;
  const float el_deg = dy * (20.f / static_cast<float>(height_)) * motion_factor_;

  clc::Vec3f up = view_up_;
  if (clc::lengthSquared(up) < 1e-12f) {
    up = {0.f, 1.f, 0.f};
  }
  up = clc::normalized(up);

  if (std::abs(az_deg) > 1e-6f) {
    position_ = clc::rotateAroundAxis(position_, focal_, up, az_deg * kDegToRad);
  }

  clc::Vec3f forward = focal_ - position_;
  if (clc::lengthSquared(forward) < 1e-12f) {
    return;
  }
  forward = clc::normalized(forward);

  clc::Vec3f elevation_axis = clc::cross(view_up_, forward);
  if (clc::lengthSquared(elevation_axis) < 1e-12f) {
    orthogonalizeViewUp();
    return;
  }
  elevation_axis = clc::normalized(elevation_axis);

  if (std::abs(el_deg) > 1e-6f) {
    position_ = clc::rotateAroundAxis(position_, focal_, elevation_axis, el_deg * kDegToRad);
  }

  orthogonalizeViewUp();
}

void TrackballCamera::pan(float dx, float dy) {
  const clc::Mat4f view = viewMatrix();
  const clc::Vec3f right = {view(0, 0), view(1, 0), view(2, 0)};
  const clc::Vec3f up = {view(0, 1), view(1, 1), view(2, 1)};
  const float scale = orbitDistance() * 0.001f;
  const clc::Vec3f delta = right * (-dx * scale) + up * (dy * scale);
  focal_ = focal_ + delta;
  position_ = position_ + delta;
}

void TrackballCamera::dolly(float delta) {
  clc::Vec3f forward = focal_ - position_;
  const float dist = clc::length(forward);
  if (dist < 1e-8f) {
    return;
  }
  forward = forward / dist;
  float new_dist = dist * std::exp(-delta * 0.001f);
  new_dist = std::clamp(new_dist, 0.05f, 1e6f);
  position_ = focal_ - forward * new_dist;
}

void TrackballCamera::spin(float dx, float dy) {
  clc::Vec3f forward = focal_ - position_;
  if (clc::lengthSquared(forward) < 1e-12f) {
    return;
  }
  forward = clc::normalized(forward);
  const float angle = (dx + dy) * 0.003f;
  clc::rotateVectorInPlace(view_up_, forward, angle);
  orthogonalizeViewUp();
}

void TrackballCamera::reset() { setOrbitFromForward(orbitDistance()); }

void TrackballCamera::flyTo(const clc::Vec3f& target, float duration_sec) {
  fly_target_ = target;
  fly_start_focal_ = focal_;
  fly_start_position_ = position_;
  fly_duration_ = std::max(0.05f, duration_sec);
  fly_distance_ = clc::length(fly_target_ - fly_start_focal_);
  fly_t_ = 0.f;
}

void TrackballCamera::tick(float dt_sec) {
  if (fly_t_ >= 1.f) {
    return;
  }
  fly_t_ = std::min(1.f, fly_t_ + dt_sec / fly_duration_);
  const auto smooth = [](float t) { return t * t * (3.f - 2.f * t); };
  const float s1 = smooth(fly_t_);
  const clc::Vec3f delta = fly_target_ - fly_start_focal_;
  focal_ = fly_start_focal_ + delta * s1;
  position_ = fly_start_position_ + delta * s1;
  if (fly_t_ >= 1.f) {
    orthogonalizeViewUp();
  }
}

clc::Mat4f TrackballCamera::viewMatrix() const {
  const clc::Vec3f eye = position_;
  const clc::Vec3f center = focal_;
  clc::Vec3f up = view_up_;
  return clc::lookAt(eye, center, up);
}

clc::Mat4f TrackballCamera::projectionMatrix() const {
  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  return clc::perspective(fovy_deg_ * kDegToRad, aspect, z_near_, z_far_);
}

}  // namespace psiviz
