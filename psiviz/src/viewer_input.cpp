// viewer_input.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/viewer_input.hpp"

#include <QtGui/QKeyEvent>

#include <cmath>

#include "psiviz/pose_gizmo.hpp"
#include "psiviz/trackball_camera.hpp"

namespace psiviz {

ViewerInputRouter::ViewerInputRouter(TrackballCamera& camera, PoseGizmo& gizmo) : camera_(camera), gizmo_(gizmo) {}

InputResult ViewerInputRouter::mousePress(const QMouseEvent& event, int width, int height) {
  InputResult result;
  if (!interaction_enabled_) {
    return result;
  }
  last_pos_ = event.position().toPoint();
  drag_button_ = event.button();
  dragging_ = true;

  if (gizmo_manipulable_ && gizmo_.enabled() && event.button() == Qt::LeftButton &&
      !event.modifiers().testFlag(Qt::ShiftModifier)) {
    const clc::Mat4f view = camera_.viewMatrix();
    const clc::Mat4f proj = camera_.projectionMatrix();
    const GizmoHit hit = gizmo_.hitTestDetailed(last_pos_.x(), last_pos_.y(), width, height, view, proj);
    if (hit.axis >= 0) {
      gizmo_.beginDrag(hit.axis, hit.mode);
      result.action = InputAction::GizmoDrag;
      result.gizmo_captured = true;
      return result;
    }
  }

  if (event.button() == Qt::LeftButton && event.modifiers().testFlag(Qt::ShiftModifier)) {
    shift_left_active_ = true;
    shift_press_pos_ = last_pos_;
    shift_dragged_ = false;
    shift_pick_pending_ = picking_enabled_;
    result.action = InputAction::CameraPan;
    return result;
  }

  if (event.button() == Qt::LeftButton) {
    result.action = InputAction::CameraRotate;
  } else if (event.button() == Qt::MiddleButton) {
    result.action = InputAction::CameraPan;
  } else if (event.button() == Qt::RightButton) {
    result.action = InputAction::CameraDolly;
  }
  return result;
}

InputResult ViewerInputRouter::mouseMove(const QMouseEvent& event, int width, int height) {
  InputResult result;
  if (!interaction_enabled_ || !dragging_) {
    return result;
  }
  const QPoint pos = event.position().toPoint();
  const float dx = static_cast<float>(pos.x() - last_pos_.x());
  const float dy = static_cast<float>(pos.y() - last_pos_.y());

  if (gizmo_.isManipulating()) {
    const clc::Mat4f view = camera_.viewMatrix();
    const clc::Mat4f proj = camera_.projectionMatrix();
    gizmo_.drag(pos.x(), pos.y(), last_pos_.x(), last_pos_.y(), width, height, view, proj);
    result.action = InputAction::GizmoDrag;
    last_pos_ = pos;
    return result;
  }

  const bool cmd = event.modifiers().testFlag(Qt::ControlModifier) || event.modifiers().testFlag(Qt::MetaModifier);
  if (drag_button_ == Qt::LeftButton && cmd) {
    camera_.spin(dx, dy);
    result.action = InputAction::CameraSpin;
  } else if (drag_button_ == Qt::LeftButton && shift_left_active_) {
    if (!shift_dragged_) {
      const float dist =
          std::hypot(static_cast<float>(pos.x() - shift_press_pos_.x()), static_cast<float>(pos.y() - shift_press_pos_.y()));
      if (dist > static_cast<float>(kPickClickMoveThresholdPx)) {
        shift_dragged_ = true;
        shift_pick_pending_ = false;
      }
    }
    camera_.pan(dx, dy);
    result.action = InputAction::CameraPan;
  } else if (drag_button_ == Qt::LeftButton) {
    camera_.rotate(dx, dy);
    result.action = InputAction::CameraRotate;
  } else if (drag_button_ == Qt::MiddleButton) {
    camera_.pan(dx, dy);
    result.action = InputAction::CameraPan;
  } else if (drag_button_ == Qt::RightButton) {
    camera_.dolly(dy * 10.f);
    result.action = InputAction::CameraDolly;
  }
  last_pos_ = pos;
  return result;
}

InputResult ViewerInputRouter::mouseRelease(const QMouseEvent& event) {
  InputResult result;
  if (shift_pick_pending_ && !shift_dragged_ && event.button() == Qt::LeftButton) {
    result.action = InputAction::PickPoint;
  }
  shift_pick_pending_ = false;
  shift_left_active_ = false;
  shift_dragged_ = false;
  if (gizmo_.isManipulating()) {
    gizmo_.endDrag();
    result.action = InputAction::GizmoDrag;
  }
  dragging_ = false;
  drag_button_ = Qt::NoButton;
  return result;
}

InputResult ViewerInputRouter::wheel(const QWheelEvent& event) {
  InputResult result;
  if (!interaction_enabled_) {
    return result;
  }
  camera_.dolly(static_cast<float>(event.angleDelta().y()));
  result.action = InputAction::CameraDolly;
  return result;
}

InputResult ViewerInputRouter::keyPress(const QKeyEvent& event) {
  InputResult result;
  const bool plus_key = event.key() == Qt::Key_Plus || event.key() == Qt::Key_Equal;
  const bool minus_key = event.key() == Qt::Key_Minus;
  if (plus_key) {
    camera_.adjustPointSize(0.5f);
    result.action = InputAction::PointSizeDelta;
    return result;
  }
  if (minus_key) {
    camera_.adjustPointSize(-0.5f);
    result.action = InputAction::PointSizeDelta;
    return result;
  }
  if (!interaction_enabled_) {
    return result;
  }
  if (event.key() == Qt::Key_R) {
    camera_.reset();
    result.action = InputAction::CameraReset;
  } else if (event.key() == Qt::Key_F) {
    result.action = InputAction::CameraFlyTo;
  } else if (event.key() == Qt::Key_H || event.key() == Qt::Key_Home) {
    result.action = InputAction::CameraFrameContent;
  }
  return result;
}

}  // namespace psiviz
