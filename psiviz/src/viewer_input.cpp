#include "psiviz/viewer_input.hpp"

#include <QtGui/QKeyEvent>

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

  if (gizmo_.enabled() && event.button() == Qt::LeftButton && !event.modifiers().testFlag(Qt::ShiftModifier)) {
    const Eigen::Matrix4f view = camera_.viewMatrix();
    const Eigen::Matrix4f proj = camera_.projectionMatrix();
    const int axis = gizmo_.hitTest(last_pos_.x(), last_pos_.y(), width, height, view, proj);
    if (axis >= 0) {
      gizmo_.beginDrag(axis, GizmoMode::Translate);
      result.action = InputAction::GizmoDrag;
      result.gizmo_captured = true;
      return result;
    }
  }

  if (event.button() == Qt::LeftButton && event.modifiers().testFlag(Qt::ShiftModifier) && picking_enabled_) {
    shift_pick_pending_ = true;
    result.action = InputAction::PickPoint;
    return result;
  }

  if (event.button() == Qt::LeftButton) {
    result.action = InputAction::CameraRotate;
  } else if (event.button() == Qt::MiddleButton ||
             (event.button() == Qt::LeftButton && event.modifiers().testFlag(Qt::ShiftModifier))) {
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
    const Eigen::Matrix4f view = camera_.viewMatrix();
    const Eigen::Matrix4f proj = camera_.projectionMatrix();
    gizmo_.drag(pos.x(), pos.y(), last_pos_.x(), last_pos_.y(), width, height, view, proj);
    result.action = InputAction::GizmoDrag;
    last_pos_ = pos;
    return result;
  }

  const bool cmd = event.modifiers().testFlag(Qt::ControlModifier) || event.modifiers().testFlag(Qt::MetaModifier);
  if (drag_button_ == Qt::LeftButton && cmd) {
    camera_.spin(dx, dy);
    result.action = InputAction::CameraSpin;
  } else if (drag_button_ == Qt::LeftButton && !event.modifiers().testFlag(Qt::ShiftModifier)) {
    camera_.rotate(dx, dy);
    result.action = InputAction::CameraRotate;
  } else if (drag_button_ == Qt::MiddleButton ||
             (drag_button_ == Qt::LeftButton && event.modifiers().testFlag(Qt::ShiftModifier))) {
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
  if (shift_pick_pending_ && event.button() == Qt::LeftButton) {
    result.action = InputAction::PickPoint;
    shift_pick_pending_ = false;
  }
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
  if (!interaction_enabled_) {
    return result;
  }
  if (event.key() == Qt::Key_R) {
    camera_.reset();
    result.action = InputAction::CameraReset;
  } else if (event.key() == Qt::Key_F) {
    result.action = InputAction::CameraFlyTo;
  } else if (event.key() == Qt::Key_Plus || event.key() == Qt::Key_Equal) {
    camera_.adjustPointSize(0.5f);
    result.action = InputAction::PointSizeDelta;
  } else if (event.key() == Qt::Key_Minus) {
    camera_.adjustPointSize(-0.5f);
    result.action = InputAction::PointSizeDelta;
  }
  return result;
}

}  // namespace psiviz
