#pragma once

#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>
#include <QtGui/QKeyEvent>

namespace psiviz {

class PoseGizmo;
class TrackballCamera;

/// High-level action the viewer should perform after handling one input event.
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

/// Outcome of routing mouse/keyboard input to camera vs gizmo vs pick.
struct InputResult {
  InputAction action{InputAction::None};
  bool gizmo_captured{false};  /// True when the gizmo consumed the event (block camera drag).
};

/// Maps Qt mouse, wheel, and key events to camera moves, gizmo drags, and pick requests.
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
  void setGizmoManipulable(bool manipulable) { gizmo_manipulable_ = manipulable; }

 private:
  TrackballCamera& camera_;
  PoseGizmo& gizmo_;
  bool interaction_enabled_{true};   /// Master switch for camera navigation.
  bool picking_enabled_{true};       /// Allow Shift+click pick.
  bool gizmo_manipulable_{true};     /// Allow dragging the pose gizmo.
  bool dragging_{false};             /// Left/middle/right drag in progress.
  Qt::MouseButton drag_button_{Qt::NoButton};
  QPoint last_pos_;                  /// Last mouse position for delta computation.
  QPoint shift_press_pos_;           /// Origin of Shift+left click (click vs drag).
  bool shift_left_active_{false};    /// Shift+left button held.
  bool shift_pick_pending_{false};   /// Waiting for release to fire pick if not dragged.
  bool shift_dragged_{false};        /// Mouse moved enough to cancel pick-on-release.
  static constexpr int kPickClickMoveThresholdPx = 4;
};

}  // namespace psiviz
