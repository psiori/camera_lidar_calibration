#include "psiviz/point_cloud_item.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QQuickWindow>
#include <QWheelEvent>

#include "psiviz/depth_picker.hpp"
#include "psiviz/point_cloud_renderer.hpp"

namespace psiviz {

class PointCloudRendererItem : public QQuickRhiItemRenderer {
 public:
  void initialize(QRhiCommandBuffer* cb) override {
    if (!rhi()) {
      return;
    }
    renderer_ = std::make_unique<PointCloudRenderer>(rhi());
    renderer_->initialize(cb);
  }

  void synchronize(QQuickRhiItem* item) override {
    auto* cloud_item = static_cast<PointCloudItem*>(item);
    if (camera_sync_from_render_) {
      cloud_item->camera_ = render_camera_;
      camera_sync_from_render_ = false;
    }
    cloud_item->updateViewport();

    {
      std::lock_guard lock(cloud_item->cloud_mutex_);
      const auto& item_cloud = cloud_item->pending_cloud_;
      const bool gpu_missing =
          renderer_ && item_cloud.point_count > 0 && (!renderer_->pipelineReady() || renderer_->pointCount() == 0);
      const bool needs_upload = cloud_item->cloud_dirty_ ||
                                (item_cloud.point_count > 0 &&
                                 last_uploaded_revision_ != cloud_item->cloud_revision_) ||
                                gpu_missing;
      if (needs_upload) {
        pending_ = item_cloud;
        cloud_item->cloud_dirty_ = false;
        upload_pending_ = true;
        pending_upload_revision_ = cloud_item->cloud_revision_;
      }
    }

    blend_weight_ = static_cast<float>(cloud_item->blend_weight_);
    scalar_min_ = static_cast<float>(cloud_item->scalar_min_);
    scalar_max_ = static_cast<float>(cloud_item->scalar_max_);
    camera_ = cloud_item->camera_;
    gizmo_ = cloud_item->gizmo_;
    dpr_ = cloud_item->window() ? cloud_item->window()->devicePixelRatio() : 1.0;
    width_ = std::max(1, static_cast<int>(cloud_item->width()));
    height_ = std::max(1, static_cast<int>(cloud_item->height()));

    if (renderer_) {
      renderer_->setBlendWeight(blend_weight_);
      renderer_->setScalarRange(scalar_min_, scalar_max_);
      renderer_->resize(width_, height_);
    }

    pick_x_ = cloud_item->pick_x_;
    pick_y_ = cloud_item->pick_y_;
    do_pick_ = cloud_item->do_pick_;
    cloud_item->do_pick_ = false;
    fly_to_ = cloud_item->fly_to_;
    cloud_item->fly_to_ = false;
    emit_pose_ = cloud_item->emit_pose_;
    cloud_item->emit_pose_ = false;

    synchronizePick(cloud_item);
  }

  void render(QRhiCommandBuffer* cb) override {
    if (!renderer_ || !renderTarget()) {
      return;
    }
    if (upload_pending_) {
      renderer_->uploadPointCloud(pending_, renderTarget());
      upload_pending_ = false;
      last_uploaded_revision_ = pending_upload_revision_;
    }
    if (do_pick_) {
      renderer_->requestPick(pick_x_, pick_y_, static_cast<float>(dpr_));
    }
    camera_.tick(1.f / 60.f);
    renderer_->render(cb, camera_, gizmo_, renderTarget());

    if (do_pick_) {
      last_depth_ = renderer_->lastPickedDepth();
      pick_ready_ = true;
      const Eigen::Matrix4f view = camera_.viewMatrix();
      const Eigen::Matrix4f proj = camera_.projectionMatrix();
      picked_point_ = DepthPicker::unproject(last_depth_, pick_x_, pick_y_, width_, height_, static_cast<float>(dpr_),
                                             view, proj);
    }
    if (fly_to_) {
      const Eigen::Matrix4f view = camera_.viewMatrix();
      const Eigen::Matrix4f proj = camera_.projectionMatrix();
      const auto pt = DepthPicker::unproject(last_depth_, pick_x_, pick_y_, width_, height_, static_cast<float>(dpr_),
                                             view, proj);
      if (pt) {
        camera_.flyTo(*pt);
      }
    }
    render_camera_ = camera_;
    if (camera_.isFlying()) {
      camera_sync_from_render_ = true;
    }
  }

  void synchronizePick(PointCloudItem* item) {
    if (!pick_ready_) {
      return;
    }
    pick_ready_ = false;
    if (picked_point_) {
      emit item->pointPicked(picked_point_->x(), picked_point_->y(), picked_point_->z());
    }
  }

  std::unique_ptr<PointCloudRenderer> renderer_;
  PointCloudData pending_;
  TrackballCamera camera_;
  PoseGizmo gizmo_;
  bool upload_pending_{false};
  std::uint64_t last_uploaded_revision_{0};
  std::uint64_t pending_upload_revision_{0};
  float blend_weight_{0.f};
  float scalar_min_{0.f};
  float scalar_max_{1.f};
  qreal dpr_{1.0};
  int width_{1};
  int height_{1};
  int pick_x_{0};
  int pick_y_{0};
  bool do_pick_{false};
  bool pick_ready_{false};
  float last_depth_{1.f};
  std::optional<Eigen::Vector3f> picked_point_;
  bool fly_to_{false};
  bool emit_pose_{false};
  TrackballCamera render_camera_;
  bool camera_sync_from_render_{false};
};

PointCloudItem::PointCloudItem(QQuickItem* parent) : QQuickRhiItem(parent) {
  setFlag(QQuickItem::ItemHasContents, true);
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setFocus(true);
}

void PointCloudItem::setBlendWeight(double w) {
  if (blend_weight_ == w) {
    return;
  }
  blend_weight_ = w;
  emit blendWeightChanged();
  update();
}

void PointCloudItem::setScalarMin(double v) {
  if (scalar_min_ == v) {
    return;
  }
  scalar_min_ = v;
  emit scalarRangeChanged();
  update();
}

void PointCloudItem::setScalarMax(double v) {
  if (scalar_max_ == v) {
    return;
  }
  scalar_max_ = v;
  emit scalarRangeChanged();
  update();
}

void PointCloudItem::setPointSize(double s) {
  camera_.setPointSize(static_cast<float>(s));
  emit pointSizeChanged();
  update();
}

void PointCloudItem::setGizmoEnabled(bool e) {
  gizmo_.setEnabled(e);
  emit gizmoEnabledChanged();
}

void PointCloudItem::setInteractionEnabled(bool e) {
  if (interaction_enabled_ == e) {
    return;
  }
  interaction_enabled_ = e;
  input_.setInteractionEnabled(e);
  emit interactionEnabledChanged();
}

bool PointCloudItem::hasPointCloudContent() const {
  std::lock_guard lock(cloud_mutex_);
  return pending_cloud_.point_count > 0;
}

std::uint64_t PointCloudItem::pointCloudPointCount() const {
  std::lock_guard lock(cloud_mutex_);
  return pending_cloud_.point_count;
}

void PointCloudItem::setPointCloudData(const QVector<float>& positions, const QVector<float>& scalars,
                                       const QVector<float>& rgb) {
  if (positions.isEmpty()) {
    return;
  }
  const auto count = static_cast<std::uint64_t>(positions.size() / 3);
  PointCloudData data;
  data.point_count = count;
  data.positions.assign(positions.begin(), positions.end());
  data.scalars.assign(scalars.begin(), scalars.end());
  data.rgb.assign(rgb.begin(), rgb.end());
  if (data.scalars.size() < count) {
    data.scalars.resize(count, 0.f);
  }
  if (data.rgb.size() < count * 3) {
    data.rgb.resize(count * 3, 0.5f);
  }
  Eigen::Vector3f bounds_min(1e30f, 1e30f, 1e30f);
  Eigen::Vector3f bounds_max(-1e30f, -1e30f, -1e30f);
  for (std::uint64_t i = 0; i < count; ++i) {
    const float x = data.positions[i * 3 + 0];
    const float y = data.positions[i * 3 + 1];
    const float z = data.positions[i * 3 + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    bounds_min.x() = std::min(bounds_min.x(), x);
    bounds_min.y() = std::min(bounds_min.y(), y);
    bounds_min.z() = std::min(bounds_min.z(), z);
    bounds_max.x() = std::max(bounds_max.x(), x);
    bounds_max.y() = std::max(bounds_max.y(), y);
    bounds_max.z() = std::max(bounds_max.z(), z);
  }
  if (bounds_min.x() <= bounds_max.x()) {
    content_bounds_min_ = bounds_min;
    content_bounds_max_ = bounds_max;
    content_bounds_valid_ = true;
  }
  {
    std::lock_guard lock(cloud_mutex_);
    pending_cloud_ = std::move(data);
    cloud_dirty_ = true;
    ++cloud_revision_;
  }
  update();
}

void PointCloudItem::updatePointCloudScalarsAndColors(const QVector<float>& scalars, const QVector<float>& rgb) {
  std::lock_guard lock(cloud_mutex_);
  if (pending_cloud_.point_count == 0) {
    return;
  }
  const auto count = pending_cloud_.point_count;
  pending_cloud_.scalars.assign(scalars.begin(), scalars.end());
  if (pending_cloud_.scalars.size() < count) {
    pending_cloud_.scalars.resize(count, 0.f);
  }
  pending_cloud_.rgb.assign(rgb.begin(), rgb.end());
  if (pending_cloud_.rgb.size() < count * 3) {
    pending_cloud_.rgb.resize(count * 3, 0.5f);
  }
  cloud_dirty_ = true;
  ++cloud_revision_;
  update();
}

void PointCloudItem::frameCameraToContent() {
  if (!content_bounds_valid_) {
    return;
  }
  camera_.fitToBounds(content_bounds_min_, content_bounds_max_);
  update();
}

QVariantList PointCloudItem::poseMatrix() const {
  QVariantList out;
  const Eigen::Matrix4f m = gizmo_.pose().matrix();
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      out.append(m(r, c));
    }
  }
  return out;
}

void PointCloudItem::setPoseMatrix(const QVariantList& values) {
  if (values.size() != 16) {
    return;
  }
  Eigen::Matrix4f m;
  int i = 0;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      m(r, c) = values[i++].toFloat();
    }
  }
  Eigen::Affine3f pose;
  pose.matrix() = m;
  gizmo_.setPose(pose);
  update();
}

QQuickRhiItemRenderer* PointCloudItem::createRenderer() {
  return new PointCloudRendererItem;
}

void PointCloudItem::updateViewport() {
  const qreal dpr = window() ? window()->devicePixelRatio() : 1.0;
  camera_.setViewport(static_cast<int>(width()), static_cast<int>(height()), static_cast<float>(dpr));
}

bool PointCloudItem::handleNavigationKey(int key, int modifiers) {
  QKeyEvent key_event(QEvent::KeyPress, static_cast<int>(key), static_cast<Qt::KeyboardModifiers>(modifiers));
  return processNavigationKey(&key_event);
}

bool PointCloudItem::processNavigationKey(QKeyEvent* key_event) {
  const InputResult result = input_.keyPress(*key_event);
  if (result.action == InputAction::PointSizeDelta) {
    emit pointSizeChanged();
    update();
    return true;
  }
  if (result.action == InputAction::CameraFlyTo) {
    pick_x_ = static_cast<int>(last_mouse_.x());
    pick_y_ = static_cast<int>(last_mouse_.y());
    do_pick_ = true;
    fly_to_ = true;
    update();
    return true;
  }
  if (result.action == InputAction::CameraReset) {
    update();
    return true;
  }
  if (result.action == InputAction::CameraFrameContent) {
    frameCameraToContent();
    return true;
  }
  return false;
}

bool PointCloudItem::event(QEvent* event) {
  const int w = std::max(1, static_cast<int>(width()));
  const int h = std::max(1, static_cast<int>(height()));

  if (event->type() == QEvent::HoverMove) {
    const auto* hover = static_cast<QHoverEvent*>(event);
    last_mouse_ = hover->position();
    return false;
  }

  if (event->type() == QEvent::KeyPress) {
    if (processNavigationKey(static_cast<QKeyEvent*>(event))) {
      return true;
    }
  }

  if (event->type() == QEvent::MouseButtonPress) {
    const auto* mouse = static_cast<QMouseEvent*>(event);
    last_mouse_ = mouse->position();
    const InputResult result = input_.mousePress(*mouse, w, h);
    if (result.action == InputAction::PickPoint) {
      pick_x_ = static_cast<int>(mouse->position().x());
      pick_y_ = static_cast<int>(mouse->position().y());
      do_pick_ = true;
    }
    if (result.action != InputAction::None) {
      update();
      return true;
    }
  }

  if (event->type() == QEvent::MouseMove) {
    const auto* mouse = static_cast<QMouseEvent*>(event);
    last_mouse_ = mouse->position();
    const InputResult result = input_.mouseMove(*mouse, w, h);
    if (result.action == InputAction::GizmoDrag) {
      emit_pose_ = true;
      emit poseChanged();
    }
    if (result.action != InputAction::None) {
      update();
      return true;
    }
  }

  if (event->type() == QEvent::MouseButtonRelease) {
    const auto* mouse = static_cast<QMouseEvent*>(event);
    const InputResult result = input_.mouseRelease(*mouse);
    if (result.action == InputAction::PickPoint) {
      pick_x_ = static_cast<int>(mouse->position().x());
      pick_y_ = static_cast<int>(mouse->position().y());
      do_pick_ = true;
    }
    if (result.action == InputAction::GizmoDrag) {
      emit poseChanged();
    }
    update();
    return true;
  }

  if (event->type() == QEvent::Wheel) {
    const auto* wheel = static_cast<QWheelEvent*>(event);
    if (input_.wheel(*wheel).action != InputAction::None) {
      update();
      return true;
    }
  }

  return QQuickRhiItem::event(event);
}

}  // namespace psiviz
