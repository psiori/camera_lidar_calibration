#pragma once

#include <clc/linalg.hpp>

namespace psiviz {

enum class GizmoMode { Translate, Rotate };

struct GizmoHit {
  int axis{-1};
  GizmoMode mode{GizmoMode::Translate};
};

class PoseGizmo {
 public:
  void setEnabled(bool enabled) { enabled_ = enabled; }
  [[nodiscard]] bool enabled() const { return enabled_; }

  void setPose(const clc::Mat4f& pose) { pose_ = pose; }
  [[nodiscard]] clc::Mat4f pose() const { return pose_; }

  void setClipScale(float s) { clip_scale_ = s; }
  [[nodiscard]] float clipScale() const { return clip_scale_; }

  /// Screen-space hit test; returns axis index 0..2, or -1 (translate handles only).
  int hitTest(int x, int y, int width, int height, const clc::Mat4f& view, const clc::Mat4f& proj) const;

  /// Hit test for translate axes and rotate rings (axis 0..2 for both modes).
  GizmoHit hitTestDetailed(int x, int y, int width, int height, const clc::Mat4f& view, const clc::Mat4f& proj)
    const;

  void beginDrag(int axis, GizmoMode mode);
  void drag(int x, int y, int prev_x, int prev_y, int width, int height, const clc::Mat4f& view,
            const clc::Mat4f& proj);
  void endDrag();

  [[nodiscard]] bool isManipulating() const { return active_axis_ >= 0; }
  [[nodiscard]] GizmoMode activeMode() const { return active_mode_; }

 private:
  bool enabled_{true};
  clc::Mat4f pose_{clc::Mat4f::identity()};
  float clip_scale_{0.12f};
  int active_axis_{-1};
  GizmoMode active_mode_{GizmoMode::Translate};
  clc::Vec3f drag_start_{};
  clc::Mat4f pose_start_{clc::Mat4f::identity()};
};

}  // namespace psiviz
