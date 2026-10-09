#pragma once

#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>
#include <QtGui/QKeyEvent>

namespace psiviz {

class PoseGizmo;
class TrackballCamera;

enum class InputAction {
  None,
  CameraRotate,
  CameraPan,
  CameraDolly,
  CameraSpin,
  CameraFlyTo,
  CameraFrameContent,
  CameraReset,
  PointSizeDelta,
  PickPoint,
  GizmoDrag,
};

struct InputResult {
  InputAction action{InputAction::None};
  bool gizmo_captured{false};
};

class ViewerInputRouter {
 public:
  ViewerInputRouter(TrackballCamera& camera, PoseGizmo& gizmo);

  InputResult mousePress(const QMouseEvent& event, int width, int height);
  InputResult mouseMove(const QMouseEvent& event, int width, int height);
  InputResult mouseRelease(const QMouseEvent& event);
  InputResult wheel(const QWheelEvent& event);
  InputResult keyPress(const QKeyEvent& event);

  void setInteractionEnabled(bool enabled) { interaction_enabled_ = enabled; }
  void setPickingEnabled(bool enabled) { picking_enabled_ = enabled; }

 private:
  TrackballCamera& camera_;
  PoseGizmo& gizmo_;
  bool interaction_enabled_{true};
  bool picking_enabled_{true};
  bool dragging_{false};
  Qt::MouseButton drag_button_{Qt::NoButton};
  QPoint last_pos_;
  QPoint shift_press_pos_;
  bool shift_left_active_{false};
  bool shift_pick_pending_{false};
  bool shift_dragged_{false};
  static constexpr int kPickClickMoveThresholdPx = 4;
};

}  // namespace psiviz
