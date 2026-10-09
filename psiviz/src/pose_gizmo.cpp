#include "psiviz/pose_gizmo.hpp"

#include <cmath>

#include <Eigen/Geometry>

namespace psiviz {

namespace {
Eigen::Vector3f axisDir(int axis) {
  if (axis == 0) {
    return Eigen::Vector3f::UnitX();
  }
  if (axis == 1) {
    return Eigen::Vector3f::UnitY();
  }
  return Eigen::Vector3f::UnitZ();
}

Eigen::Vector2f project(const Eigen::Vector3f& p, const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj, int w,
                        int h) {
  const Eigen::Vector4f clip = proj * view * Eigen::Vector4f(p.x(), p.y(), p.z(), 1.f);
  if (std::fabs(clip.w()) < 1e-6f) {
    return {0.f, 0.f};
  }
  const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
  return {(ndc.x() * 0.5f + 0.5f) * static_cast<float>(w), (1.f - (ndc.y() * 0.5f + 0.5f)) * static_cast<float>(h)};
}
}  // namespace

int PoseGizmo::hitTest(int x, int y, int width, int height, const Eigen::Matrix4f& view,
                       const Eigen::Matrix4f& proj) const {
  if (!enabled_) {
    return -1;
  }
  const Eigen::Vector3f origin = pose_.translation();
  float best = 20.f;
  int best_axis = -1;
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3f world = origin + pose_.linear() * axisDir(axis) * clip_scale_;
    const Eigen::Vector2f screen = project(world, view, proj, width, height);
    const float d = (screen - Eigen::Vector2f(static_cast<float>(x), static_cast<float>(y))).norm();
    if (d < best) {
      best = d;
      best_axis = axis;
    }
  }
  return best_axis;
}

void PoseGizmo::beginDrag(int axis, GizmoMode mode) {
  active_axis_ = axis;
  active_mode_ = mode;
  pose_start_ = pose_;
  drag_start_ = pose_.translation();
}

void PoseGizmo::drag(int x, int y, int prev_x, int prev_y, int width, int height, const Eigen::Matrix4f& view,
                     const Eigen::Matrix4f& proj) {
  if (active_axis_ < 0) {
    return;
  }
  if (active_mode_ == GizmoMode::Translate) {
    const Eigen::Vector3f axis_w = pose_start_.linear() * axisDir(active_axis_);
    const Eigen::Vector2f p0 = project(drag_start_, view, proj, width, height);
    const Eigen::Vector2f p1 = project(drag_start_ + axis_w * 0.1f, view, proj, width, height);
    const Eigen::Vector2f dir = (p1 - p0).normalized();
    const float delta = static_cast<float>(x - prev_x) * dir.x() + static_cast<float>(y - prev_y) * dir.y();
    pose_.translation() = pose_start_.translation() + axis_w * delta * 0.01f;
  } else {
    const float angle = static_cast<float>(x - prev_x) * 0.01f;
    const Eigen::Vector3f axis_w = pose_start_.linear() * axisDir(active_axis_);
    pose_ = pose_start_;
    pose_.linear() = Eigen::AngleAxisf(angle, axis_w.normalized()).toRotationMatrix() * pose_start_.linear();
  }
}

void PoseGizmo::endDrag() {
  active_axis_ = -1;
}

}  // namespace psiviz
