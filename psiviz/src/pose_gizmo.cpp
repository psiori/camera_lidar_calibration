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
  const GizmoHit hit = hitTestDetailed(x, y, width, height, view, proj);
  return hit.mode == GizmoMode::Translate ? hit.axis : -1;
}

GizmoHit PoseGizmo::hitTestDetailed(int x, int y, int width, int height, const Eigen::Matrix4f& view,
                                  const Eigen::Matrix4f& proj) const {
  GizmoHit best_hit;
  if (!enabled_) {
    return best_hit;
  }
  const Eigen::Vector3f origin = pose_.translation();
  const Eigen::Vector2f cursor(static_cast<float>(x), static_cast<float>(y));
  float best_dist = 20.f;

  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3f world = origin + pose_.linear() * axisDir(axis) * clip_scale_;
    const Eigen::Vector2f screen = project(world, view, proj, width, height);
    const float d = (screen - cursor).norm();
    if (d < best_dist) {
      best_dist = d;
      best_hit.axis = axis;
      best_hit.mode = GizmoMode::Translate;
    }
  }

  const float ring_scale = clip_scale_ * 0.7f;
  const int ring_segments = 24;
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3f n = pose_.linear() * axisDir(axis);
    Eigen::Vector3f tangent = pose_.linear() * axisDir((axis + 1) % 3);
    if (tangent.norm() < 1e-6f) {
      continue;
    }
    tangent.normalize();
    const Eigen::Vector3f bitangent = n.cross(tangent).normalized();
    for (int seg_i = 0; seg_i < ring_segments; ++seg_i) {
      constexpr float kPi = 3.14159265f;
      const float a0 = static_cast<float>(seg_i) / static_cast<float>(ring_segments) * 2.f * kPi;
      const float a1 = static_cast<float>(seg_i + 1) / static_cast<float>(ring_segments) * 2.f * kPi;
      const Eigen::Vector3f p0 = origin + ring_scale * (std::cos(a0) * tangent + std::sin(a0) * bitangent);
      const Eigen::Vector3f p1 = origin + ring_scale * (std::cos(a1) * tangent + std::sin(a1) * bitangent);
      const Eigen::Vector2f s0 = project(p0, view, proj, width, height);
      const Eigen::Vector2f s1 = project(p1, view, proj, width, height);
      const Eigen::Vector2f seg_dir = s1 - s0;
      const float len2 = seg_dir.squaredNorm();
      float t = 0.f;
      if (len2 > 1e-6f) {
        t = std::clamp((cursor - s0).dot(seg_dir) / len2, 0.f, 1.f);
      }
      const Eigen::Vector2f closest = s0 + t * seg_dir;
      const float d = (closest - cursor).norm();
      if (d < best_dist) {
        best_dist = d;
        best_hit.axis = axis;
        best_hit.mode = GizmoMode::Rotate;
      }
    }
  }
  return best_hit;
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
