#pragma once

#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace psiviz {

/// View navigation matching VTK vtkInteractorStyleTrackballCamera (Azimuth/Elevation about focal).
class TrackballCamera {
 public:
  TrackballCamera();

  void setViewport(int width, int height, float device_pixel_ratio);
  void setPerspective(float fovy_deg, float z_near, float z_far);
  void fitToBounds(const Eigen::Vector3f& min, const Eigen::Vector3f& max);

  void rotate(float dx, float dy);
  void pan(float dx, float dy);
  void dolly(float delta);
  void spin(float dx, float dy);
  void reset();
  void flyTo(const Eigen::Vector3f& target, float duration_sec = 0.35f);
  void tick(float dt_sec);
  [[nodiscard]] bool isFlying() const { return fly_t_ < 1.f; }

  [[nodiscard]] Eigen::Matrix4f viewMatrix() const;
  [[nodiscard]] Eigen::Matrix4f projectionMatrix() const;
  [[nodiscard]] Eigen::Vector3f focalPoint() const { return focal_; }
  [[nodiscard]] float pointSize() const { return point_size_; }
  void setPointSize(float size) { point_size_ = size; }
  void adjustPointSize(float delta) { point_size_ = std::max(1.f, point_size_ + delta); }

 private:
  void orthogonalizeViewUp();
  void setOrbitFromForward(float orbit_distance);
  [[nodiscard]] float orbitDistance() const;

  Eigen::Vector3f position_{0.f, -2.96f, -9.55f};
  Eigen::Vector3f focal_{0.f, 0.f, 0.f};
  Eigen::Vector3f view_up_{0.f, 1.f, 0.f};
  float point_size_{2.f};
  int width_{1};
  int height_{1};
  float dpr_{1.f};
  float fovy_deg_{60.f};
  float z_near_{0.01f};
  float z_far_{1000.f};
  float motion_factor_{10.f};

  Eigen::Vector3f fly_target_{};
  Eigen::Vector3f fly_start_focal_{};
  Eigen::Vector3f fly_start_position_{};
  float fly_t_{1.f};
  float fly_duration_{0.35f};
  float fly_distance_{0.f};
};

}  // namespace psiviz
