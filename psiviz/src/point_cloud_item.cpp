// point_cloud_item.cpp — psiviz rendering, input, or geometry implementation.

#include "psiviz/point_cloud_item.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

#include <QKeyEvent>
#include <QLoggingCategory>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPointer>
#include <QQuickWindow>
#include <QWheelEvent>
#include <rhi/qrhi.h>

#include "psiviz/point_cloud_renderer.hpp"

namespace psiviz {

namespace {
Q_LOGGING_CATEGORY(psivizPickLog, "psiviz.pick")
}  // namespace

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
    item_ = cloud_item;
    if (camera_sync_from_render_) {
      const float point_size = cloud_item->camera_.pointSize();
      cloud_item->camera_ = render_camera_;
      cloud_item->camera_.setPointSize(point_size);
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
      renderer_->setCalibrationSceneOverlays(cloud_item->scene_overlays_);
    }

    // Deliver prior-frame pick results before overwriting pick_emit_signal_.
    synchronizePick(cloud_item);

    pick_x_ = cloud_item->pick_x_;
    pick_y_ = cloud_item->pick_y_;
    do_pick_ = cloud_item->do_pick_;
    cloud_item->do_pick_ = false;
    fly_to_ = cloud_item->fly_to_;
    cloud_item->fly_to_ = false;
    pick_emit_signal_ = cloud_item->pick_emit_signal_;
    cloud_item->pick_emit_signal_ = false;
    emit_pose_ = cloud_item->emit_pose_;
    cloud_item->emit_pose_ = false;

    {
      std::lock_guard lock(cloud_item->markers_mutex_);
      if (cloud_item->markers_dirty_) {
        cloud_item->rebuildSphereMarkersFromPositions();
        cloud_item->markers_dirty_ = false;
      }
      if (renderer_) {
        renderer_->setSphereMarkers(cloud_item->sphere_markers_);
      }
    }
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
      pick_resolve_request_id_ =
          renderer_->requestPick(pick_x_, pick_y_, static_cast<float>(dpr_), camera_.pointSize());
      pick_pending_resolve_ = true;
      pick_resolve_x_ = pick_x_;
      pick_resolve_y_ = pick_y_;
      pick_resolve_fly_to_ = fly_to_;
      pick_resolve_emit_signal_ = pick_emit_signal_;
      do_pick_ = false;
      fly_to_ = false;
      pick_emit_signal_ = false;
    }
    const bool flying_before = camera_.isFlying();
    camera_.tick(1.f / 60.f);
    renderer_->render(cb, camera_, gizmo_, renderTarget());

    if (pick_pending_resolve_) {
      std::uint32_t point_id = 0;
      bool consumed = renderer_->tryConsumePickResult(pick_resolve_request_id_, point_id);
      if (!consumed && rhi()) {
        rhi()->finish();
        consumed = renderer_->tryConsumePickResult(pick_resolve_request_id_, point_id);
      }
      if (consumed) {
        resolvePendingPick(point_id);
      }
    }
    render_camera_ = camera_;
    if (flying_before || camera_.isFlying()) {
      camera_sync_from_render_ = true;
    }
    requestItemUpdate();
  }

  void resolvePendingPick(std::uint32_t point_id) {
    pick_pending_resolve_ = false;
    last_picked_point_id_ = point_id;
    pick_ready_ = true;
    last_pick_x_ = pick_resolve_x_;
    last_pick_y_ = pick_resolve_y_;
    picked_point_.reset();
    if (point_id > 0 && point_id <= pending_.point_count) {
      const std::uint64_t i = point_id - 1;
      picked_point_ = clc::Vec3f{pending_.positions[i * 3 + 0], pending_.positions[i * 3 + 1],
                                 pending_.positions[i * 3 + 2]};
    }
    if (picked_point_) {
      if (pick_resolve_fly_to_) {
        camera_.flyTo(*picked_point_);
        qCInfo(psivizPickLog) << "fly-to focal ->" << picked_point_->x << picked_point_->y << picked_point_->z
                              << "at pixel" << pick_resolve_x_ << pick_resolve_y_ << "pointId" << point_id;
      }
    } else {
      qCInfo(psivizPickLog) << "pick missed at pixel" << pick_resolve_x_ << pick_resolve_y_ << "pointId" << point_id
                            << "points" << static_cast<qulonglong>(pending_.point_count);
    }
    pick_resolve_fly_to_ = false;
    if (item_) {
      QMetaObject::invokeMethod(item_, "update", Qt::QueuedConnection);
    }
  }

  void requestItemUpdate() {
    if (item_ && (camera_.isFlying() || camera_sync_from_render_ || pick_pending_resolve_)) {
      QMetaObject::invokeMethod(item_, "update", Qt::QueuedConnection);
    }
  }

  void synchronizePick(PointCloudItem* item) {
    if (!pick_ready_) {
      return;
    }
    pick_ready_ = false;
    if (picked_point_) {
      const auto& p = *picked_point_;
      qCInfo(psivizPickLog).noquote()
          << QStringLiteral("picked xyz=(%1,%2,%3) pixel=(%4,%5) pointId=%6")
                 .arg(p.x, 0, 'f', 4)
                 .arg(p.y, 0, 'f', 4)
                 .arg(p.z, 0, 'f', 4)
                 .arg(last_pick_x_)
                 .arg(last_pick_y_)
                 .arg(last_picked_point_id_);
      if (pick_resolve_emit_signal_) {
        emit item->pointPicked(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
      }
    } else {
      qCInfo(psivizPickLog).noquote() << QStringLiteral("pick failed pixel=(%1,%2) pointId=%3")
                                            .arg(last_pick_x_)
                                            .arg(last_pick_y_)
                                            .arg(last_picked_point_id_);
    }
    pick_resolve_emit_signal_ = false;
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
  std::uint32_t last_picked_point_id_{0};
  std::optional<clc::Vec3f> picked_point_;
  bool fly_to_{false};
  bool pick_emit_signal_{false};
  int last_pick_x_{0};
  int last_pick_y_{0};
  bool pick_pending_resolve_{false};
  std::uint64_t pick_resolve_request_id_{0};
  int pick_resolve_x_{0};
  int pick_resolve_y_{0};
  bool pick_resolve_fly_to_{false};
  bool pick_resolve_emit_signal_{false};
  bool emit_pose_{false};
  TrackballCamera render_camera_;
  bool camera_sync_from_render_{false};
  QPointer<PointCloudItem> item_;
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

void PointCloudItem::bumpPointSize(double delta) {
  camera_.adjustPointSize(static_cast<float>(delta));
  emit pointSizeChanged();
  update();
}

void PointCloudItem::setPointerPosition(double x, double y) {
  last_mouse_ = QPointF(x, y);
  pointer_position_valid_ = true;
}

float PointCloudItem::effectiveMarkerSphereRadius() const {
  if (marker_sphere_radius_ > 0.) {
    return static_cast<float>(marker_sphere_radius_);
  }
  return 0.05f;
}

void PointCloudItem::rebuildSphereMarkersFromPositions() {
  sphere_markers_.clear();
  const float radius = effectiveMarkerSphereRadius();
  const int count = static_cast<int>(marker_positions_xyz_.size() / 3);
  for (int i = 0; i < count; ++i) {
    SphereMarker marker;
    marker.center = clc::Vec3f{static_cast<float>(marker_positions_xyz_[i * 3 + 0]),
                               static_cast<float>(marker_positions_xyz_[i * 3 + 1]),
                               static_cast<float>(marker_positions_xyz_[i * 3 + 2])};
    marker.radius = radius;
    marker.color = {1.f, 0.45f, 0.f};
    marker.alpha = 1.f;
    sphere_markers_.push_back(marker);
  }
  if (pending_marker_) {
    SphereMarker marker;
    marker.center = *pending_marker_;
    marker.radius = radius * 1.35f;
    marker.color = {0.15f, 0.95f, 1.f};
    marker.alpha = 1.f;
    sphere_markers_.push_back(marker);
  }
}

void PointCloudItem::setSphereMarkerPositions(const QVector<double>& positions_xyz) {
  if (positions_xyz.size() % 3 != 0) {
    return;
  }
  {
    std::lock_guard lock(markers_mutex_);
    marker_positions_xyz_ = positions_xyz;
    markers_dirty_ = true;
  }
  update();
}

void PointCloudItem::setPendingSphereMarker(double x, double y, double z) {
  {
    std::lock_guard lock(markers_mutex_);
    pending_marker_ = clc::Vec3f{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    markers_dirty_ = true;
  }
  update();
}

void PointCloudItem::clearPendingSphereMarker() {
  {
    std::lock_guard lock(markers_mutex_);
    pending_marker_.reset();
    markers_dirty_ = true;
  }
  update();
}

void PointCloudItem::clearSphereMarkers() {
  {
    std::lock_guard lock(markers_mutex_);
    marker_positions_xyz_.clear();
    pending_marker_.reset();
    sphere_markers_.clear();
    markers_dirty_ = true;
  }
  update();
}

void PointCloudItem::setMarkerSphereRadius(double radius) {
  if (marker_sphere_radius_ == radius) {
    return;
  }
  marker_sphere_radius_ = radius;
  {
    std::lock_guard lock(markers_mutex_);
    markers_dirty_ = true;
  }
  emit markerSphereRadiusChanged();
  update();
}

void PointCloudItem::flyToUnderCursor() {
  if (!interaction_enabled_) {
    return;
  }
  const int w = std::max(1, static_cast<int>(width()));
  const int h = std::max(1, static_cast<int>(height()));
  if (pointer_position_valid_) {
    pick_x_ = static_cast<int>(last_mouse_.x());
    pick_y_ = static_cast<int>(last_mouse_.y());
  } else {
    pick_x_ = w / 2;
    pick_y_ = h / 2;
  }
  pick_x_ = std::clamp(pick_x_, 0, w - 1);
  pick_y_ = std::clamp(pick_y_, 0, h - 1);
  do_pick_ = true;
  fly_to_ = true;
  pick_emit_signal_ = false;
  qCInfo(psivizPickLog) << "fly-to requested at pixel" << pick_x_ << pick_y_;
  update();
}

void PointCloudItem::setGizmoEnabled(bool e) {
  gizmo_.setEnabled(e);
  emit gizmoEnabledChanged();
  update();
}

void PointCloudItem::setSceneOverlayTriads(bool camera_triad, bool lidar_triad) {
  scene_overlays_.camera_triad = camera_triad;
  scene_overlays_.lidar_triad = lidar_triad;
  update();
}

void PointCloudItem::setCameraAxisLength(double meters) {
  gizmo_.setClipScale(static_cast<float>(meters));
  update();
}

void PointCloudItem::setCameraFrustumIntrinsics(bool valid, double fx, double fy, double cx, double cy,
                                                double image_width, double image_height, double depth_meters) {
  scene_overlays_.frustum.valid = valid;
  scene_overlays_.frustum.fx = static_cast<float>(fx);
  scene_overlays_.frustum.fy = static_cast<float>(fy);
  scene_overlays_.frustum.cx = static_cast<float>(cx);
  scene_overlays_.frustum.cy = static_cast<float>(cy);
  scene_overlays_.frustum.image_width = static_cast<float>(image_width);
  scene_overlays_.frustum.image_height = static_cast<float>(image_height);
  scene_overlays_.frustum.depth_m = static_cast<float>(depth_meters);
  update();
}

void PointCloudItem::setGizmoManipulable(bool m) {
  if (gizmo_manipulable_ == m) {
    return;
  }
  gizmo_manipulable_ = m;
  input_.setGizmoManipulable(m);
  emit gizmoManipulableChanged();
}

void PointCloudItem::setInteractionEnabled(bool e) {
  if (interaction_enabled_ == e) {
    return;
  }
  interaction_enabled_ = e;
  input_.setInteractionEnabled(e);
  emit interactionEnabledChanged();
}

void PointCloudItem::setPickingEnabled(bool e) {
  if (picking_enabled_ == e) {
    return;
  }
  picking_enabled_ = e;
  input_.setPickingEnabled(e);
  emit pickingEnabledChanged();
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
    data.rgb.resize(count * 3, 0.f);
  }
  clc::Vec3f bounds_min{1e30f, 1e30f, 1e30f};
  clc::Vec3f bounds_max{-1e30f, -1e30f, -1e30f};
  for (std::uint64_t i = 0; i < count; ++i) {
    const float x = data.positions[i * 3 + 0];
    const float y = data.positions[i * 3 + 1];
    const float z = data.positions[i * 3 + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    bounds_min.x = std::min(bounds_min.x, x);
    bounds_min.y = std::min(bounds_min.y, y);
    bounds_min.z = std::min(bounds_min.z, z);
    bounds_max.x = std::max(bounds_max.x, x);
    bounds_max.y = std::max(bounds_max.y, y);
    bounds_max.z = std::max(bounds_max.z, z);
  }
  if (bounds_min.x <= bounds_max.x) {
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
    pending_cloud_.rgb.resize(count * 3, 0.f);
  }
  cloud_dirty_ = true;
  ++cloud_revision_;
  update();
}

void PointCloudItem::requestPickAt(int x, int y) {
  const int w = std::max(1, static_cast<int>(width()));
  const int h = std::max(1, static_cast<int>(height()));
  pick_x_ = std::clamp(x, 0, w - 1);
  pick_y_ = std::clamp(y, 0, h - 1);
  do_pick_ = true;
  pick_emit_signal_ = true;
  fly_to_ = false;
  qCInfo(psivizPickLog) << "programmatic pick at pixel" << pick_x_ << pick_y_;
  update();
}

void PointCloudItem::frameCameraToContent() {
  if (!content_bounds_valid_) {
    return;
  }
  camera_.fitToBounds(content_bounds_min_, content_bounds_max_);
  update();
}

void PointCloudItem::recomputeClipPlanes() {
  camera_.recomputeClipPlanes();
  update();
}

QVariantList PointCloudItem::poseMatrix() const {
  QVariantList out;
  const clc::Mat4f m = gizmo_.pose();
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
  clc::Mat4f m = clc::Mat4f::identity();
  int i = 0;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      m(r, c) = values[i++].toFloat();
    }
  }
  gizmo_.setPose(m);
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
    const int w = std::max(1, static_cast<int>(width()));
    const int h = std::max(1, static_cast<int>(height()));
    if (pointer_position_valid_) {
      pick_x_ = static_cast<int>(last_mouse_.x());
      pick_y_ = static_cast<int>(last_mouse_.y());
    } else {
      pick_x_ = w / 2;
      pick_y_ = h / 2;
    }
    pick_x_ = std::clamp(pick_x_, 0, w - 1);
    pick_y_ = std::clamp(pick_y_, 0, h - 1);
    do_pick_ = true;
    fly_to_ = true;
    pick_emit_signal_ = false;
    qCInfo(psivizPickLog) << "fly-to requested at pixel" << pick_x_ << pick_y_;
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
  if (result.action == InputAction::CameraRecomputeClipPlanes) {
    recomputeClipPlanes();
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
    pointer_position_valid_ = true;
    return false;
  }

  if (event->type() == QEvent::KeyPress) {
    if (processNavigationKey(static_cast<QKeyEvent*>(event))) {
      return true;
    }
  }

  if (event->type() == QEvent::MouseButtonPress) {
    const auto* mouse = static_cast<QMouseEvent*>(event);
    forceActiveFocus(Qt::MouseFocusReason);
    last_mouse_ = mouse->position();
    pointer_position_valid_ = true;
    const InputResult result = input_.mousePress(*mouse, w, h);
    if (result.action == InputAction::PickPoint) {
      return true;
    }
    if (result.action != InputAction::None) {
      update();
      return true;
    }
  }

  if (event->type() == QEvent::MouseMove) {
    const auto* mouse = static_cast<QMouseEvent*>(event);
    last_mouse_ = mouse->position();
    pointer_position_valid_ = true;
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
      pick_emit_signal_ = true;
      fly_to_ = false;
      qCInfo(psivizPickLog) << "shift+click pick at pixel" << pick_x_ << pick_y_;
    }
    if (result.action == InputAction::GizmoDrag) {
      emit poseChanged();
    }
    if (result.action != InputAction::None) {
      update();
      return true;
    }
    return QQuickRhiItem::event(event);
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
