// pose_gizmo.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/pose_gizmo.hpp"

#include <algorithm>
#include <cmath>

namespace psiviz {

namespace {

clc::Vec3f axisDir(int axis) {
  if (axis == 0) {
    return {1.f, 0.f, 0.f};
  }
  if (axis == 1) {
    return {0.f, 1.f, 0.f};
  }
  return {0.f, 0.f, 1.f};
}

clc::Vec2f project(const clc::Vec3f& p, const clc::Mat4f& view, const clc::Mat4f& proj, int w, int h) {
  const clc::Vec4f clip = clc::mulMat4Vec4(proj, clc::mulMat4Vec4(view, {p.x, p.y, p.z, 1.f}));
  if (std::fabs(clip.w) < 1e-6f) {
    return {0.f, 0.f};
  }
  const float inv_w = 1.f / clip.w;
  const clc::Vec3f ndc{clip.x * inv_w, clip.y * inv_w, clip.z * inv_w};
  return {(ndc.x * 0.5f + 0.5f) * static_cast<float>(w), (1.f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(h)};
}

clc::Mat4f applyDeltaRotation(const clc::Mat4f& delta, const clc::Mat4f& pose) {
  clc::Mat4f out = pose;
  for (int col = 0; col < 3; ++col) {
    clc::setRotationColumn(out, col, clc::transformVector(delta, clc::rotationColumn(pose, col)));
  }
  return out;
}

}  // namespace

int PoseGizmo::hitTest(int x, int y, int width, int height, const clc::Mat4f& view, const clc::Mat4f& proj) const {
  const GizmoHit hit = hitTestDetailed(x, y, width, height, view, proj);
  return hit.mode == GizmoMode::Translate ? hit.axis : -1;
}

GizmoHit PoseGizmo::hitTestDetailed(int x, int y, int width, int height, const clc::Mat4f& view,
                                  const clc::Mat4f& proj) const {
  GizmoHit best_hit;
  if (!enabled_) {
    return best_hit;
  }
  const clc::Vec3f origin = clc::translation(pose_);
  const clc::Vec2f cursor{static_cast<float>(x), static_cast<float>(y)};
  float best_dist = 20.f;

  for (int axis = 0; axis < 3; ++axis) {
    const clc::Vec3f world =
        origin + clc::transformVector(pose_, axisDir(axis)) * clip_scale_;
    const clc::Vec2f screen = project(world, view, proj, width, height);
    const float d = clc::length(screen - cursor);
    if (d < best_dist) {
      best_dist = d;
      best_hit.axis = axis;
      best_hit.mode = GizmoMode::Translate;
    }
  }

  const float ring_scale = clip_scale_ * 0.7f;
  const int ring_segments = 24;
  for (int axis = 0; axis < 3; ++axis) {
    const clc::Vec3f n = clc::transformVector(pose_, axisDir(axis));
    clc::Vec3f tangent = clc::transformVector(pose_, axisDir((axis + 1) % 3));
    if (clc::length(tangent) < 1e-6f) {
      continue;
    }
    tangent = clc::normalized(tangent);
    const clc::Vec3f bitangent = clc::normalized(clc::cross(n, tangent));
    for (int seg_i = 0; seg_i < ring_segments; ++seg_i) {
      constexpr float kPi = 3.14159265f;
      const float a0 = static_cast<float>(seg_i) / static_cast<float>(ring_segments) * 2.f * kPi;
      const float a1 = static_cast<float>(seg_i + 1) / static_cast<float>(ring_segments) * 2.f * kPi;
      const clc::Vec3f p0 = origin + ring_scale * (std::cos(a0) * tangent + std::sin(a0) * bitangent);
      const clc::Vec3f p1 = origin + ring_scale * (std::cos(a1) * tangent + std::sin(a1) * bitangent);
      const clc::Vec2f s0 = project(p0, view, proj, width, height);
      const clc::Vec2f s1 = project(p1, view, proj, width, height);
      const clc::Vec2f seg_dir = s1 - s0;
      const float len2 = clc::lengthSquared(seg_dir);
      float t = 0.f;
      if (len2 > 1e-6f) {
        t = std::clamp(clc::dot(cursor - s0, seg_dir) / len2, 0.f, 1.f);
      }
      const clc::Vec2f closest = s0 + seg_dir * t;
      const float d = clc::length(closest - cursor);
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
  drag_start_ = clc::translation(pose_);
}

void PoseGizmo::drag(int x, int y, int prev_x, int prev_y, int width, int height, const clc::Mat4f& view,
                     const clc::Mat4f& proj) {
  if (active_axis_ < 0) {
    return;
  }
  if (active_mode_ == GizmoMode::Translate) {
    const clc::Vec3f axis_w = clc::transformVector(pose_start_, axisDir(active_axis_));
    const clc::Vec2f p0 = project(drag_start_, view, proj, width, height);
    const clc::Vec2f p1 = project(drag_start_ + axis_w * 0.1f, view, proj, width, height);
    const clc::Vec2f dir = clc::normalized(p1 - p0);
    const float delta =
        static_cast<float>(x - prev_x) * dir.x + static_cast<float>(y - prev_y) * dir.y;
    pose_ = pose_start_;
    clc::setTranslation(pose_, clc::translation(pose_start_) + axis_w * (delta * 0.01f));
  } else {
    const float angle = static_cast<float>(x - prev_x) * 0.01f;
    const clc::Vec3f axis_w = clc::transformVector(pose_start_, axisDir(active_axis_));
    const clc::Mat4f delta = clc::rotationAxisAngle(clc::normalized(axis_w), angle);
    pose_ = applyDeltaRotation(delta, pose_start_);
  }
}

void PoseGizmo::endDrag() { active_axis_ = -1; }

}  // namespace psiviz
