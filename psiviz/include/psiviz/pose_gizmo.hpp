#pragma once

#include <Eigen/Geometry>

namespace psiviz {

enum class GizmoMode { Translate, Rotate };

class PoseGizmo {
 public:
  void setEnabled(bool enabled) { enabled_ = enabled; }
  [[nodiscard]] bool enabled() const { return enabled_; }

  void setPose(const Eigen::Affine3f& pose) { pose_ = pose; }
  [[nodiscard]] Eigen::Affine3f pose() const { return pose_; }

  void setClipScale(float s) { clip_scale_ = s; }
  [[nodiscard]] float clipScale() const { return clip_scale_; }

  /// Screen-space hit test; returns mode and axis index 0..2, or -1.
  int hitTest(int x, int y, int width, int height, const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj) const;

  void beginDrag(int axis, GizmoMode mode);
  void drag(int x, int y, int prev_x, int prev_y, int width, int height, const Eigen::Matrix4f& view,
            const Eigen::Matrix4f& proj);
  void endDrag();

  [[nodiscard]] bool isManipulating() const { return active_axis_ >= 0; }
  [[nodiscard]] GizmoMode activeMode() const { return active_mode_; }

 private:
  bool enabled_{true};
  Eigen::Affine3f pose_{Eigen::Affine3f::Identity()};
  float clip_scale_{0.12f};
  int active_axis_{-1};
  GizmoMode active_mode_{GizmoMode::Translate};
  Eigen::Vector3f drag_start_{};
  Eigen::Affine3f pose_start_{};
};

}  // namespace psiviz
