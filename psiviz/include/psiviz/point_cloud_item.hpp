#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include <Eigen/Core>

#include <QPointF>
#include <QQuickRhiItem>

class QKeyEvent;
#include <QtQml/qqmlregistration.h>

#include "psiviz/point_cloud_buffers.hpp"
#include "psiviz/pose_gizmo.hpp"
#include "psiviz/scene_shapes.hpp"
#include "psiviz/trackball_camera.hpp"
#include "psiviz/viewer_input.hpp"

namespace psiviz {

class PointCloudItem : public QQuickRhiItem {
  Q_OBJECT
  QML_NAMED_ELEMENT(PointCloudScene)
  Q_PROPERTY(double blendWeight READ blendWeight WRITE setBlendWeight NOTIFY blendWeightChanged)
  Q_PROPERTY(double scalarMin READ scalarMin WRITE setScalarMin NOTIFY scalarRangeChanged)
  Q_PROPERTY(double scalarMax READ scalarMax WRITE setScalarMax NOTIFY scalarRangeChanged)
  Q_PROPERTY(double pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
  Q_PROPERTY(bool gizmoEnabled READ gizmoEnabled WRITE setGizmoEnabled NOTIFY gizmoEnabledChanged)
  Q_PROPERTY(bool interactionEnabled READ interactionEnabled WRITE setInteractionEnabled NOTIFY interactionEnabledChanged)
  Q_PROPERTY(double markerSphereRadius READ markerSphereRadius WRITE setMarkerSphereRadius NOTIFY markerSphereRadiusChanged)

 public:
  explicit PointCloudItem(QQuickItem* parent = nullptr);

  double blendWeight() const { return blend_weight_; }
  void setBlendWeight(double w);
  double scalarMin() const { return scalar_min_; }
  void setScalarMin(double v);
  double scalarMax() const { return scalar_max_; }
  void setScalarMax(double v);
  double pointSize() const { return camera_.pointSize(); }
  void setPointSize(double s);
  bool gizmoEnabled() const { return gizmo_.enabled(); }
  void setGizmoEnabled(bool e);
  bool interactionEnabled() const { return interaction_enabled_; }
  void setInteractionEnabled(bool e);
  [[nodiscard]] bool hasPointCloudContent() const;
  [[nodiscard]] std::uint64_t pointCloudPointCount() const;

  Q_INVOKABLE void setPointCloudData(const QVector<float>& positions, const QVector<float>& scalars,
                                     const QVector<float>& rgb);
  /// Updates scalar/RGB buffers only; keeps camera and positions unchanged.
  Q_INVOKABLE void updatePointCloudScalarsAndColors(const QVector<float>& scalars, const QVector<float>& rgb);
  Q_INVOKABLE void frameCameraToContent();
  Q_INVOKABLE QVariantList poseMatrix() const;
  Q_INVOKABLE void setPoseMatrix(const QVariantList& values);
  /// Navigation keys (`f`, `+`, `-`, `r`, `h`) for QML forwarding when the pointer is over the viewport.
  Q_INVOKABLE bool handleNavigationKey(int key, int modifiers);
  Q_INVOKABLE void bumpPointSize(double delta);
  /// Depth pick at last pointer position (or viewport center) and animate focal point there (`f`).
  Q_INVOKABLE void flyToUnderCursor();
  Q_INVOKABLE void setPointerPosition(double x, double y);
  /// Replace pick markers with spheres at each (x,y,z) triple in `positions_xyz` (length must be a multiple of 3).
  Q_INVOKABLE void setSphereMarkerPositions(const QVector<double>& positions_xyz);
  Q_INVOKABLE void setPendingSphereMarker(double x, double y, double z);
  Q_INVOKABLE void clearPendingSphereMarker();
  Q_INVOKABLE void clearSphereMarkers();
  /// Same pick path as Shift+click at viewport pixel (x, y); emits `pointPicked` when complete.
  Q_INVOKABLE void requestPickAt(int x, int y);
  /// Force one point to bright red (scalar max + RGB) and upload to the GPU.
  Q_INVOKABLE void highlightPointIndex(qint64 index);

  double markerSphereRadius() const { return marker_sphere_radius_; }
  void setMarkerSphereRadius(double radius);

 signals:
  void blendWeightChanged();
  void scalarRangeChanged();
  void pointSizeChanged();
  void gizmoEnabledChanged();
  void interactionEnabledChanged();
  void markerSphereRadiusChanged();
  void pointPicked(qint64 point_index);
  void poseChanged();

 protected:
  QQuickRhiItemRenderer* createRenderer() override;
  bool event(QEvent* event) override;

 private:
  friend class PointCloudRendererItem;
  void updateViewport();
  bool processNavigationKey(QKeyEvent* key_event);
  [[nodiscard]] float effectiveMarkerSphereRadius() const;
  void rebuildSphereMarkersFromPositions();

  int pick_x_{0};
  int pick_y_{0};
  bool do_pick_{false};
  bool fly_to_{false};
  bool pick_emit_signal_{false};
  bool emit_pose_{false};
  QPointF last_mouse_;
  bool pointer_position_valid_{false};

  TrackballCamera camera_;
  PoseGizmo gizmo_;
  ViewerInputRouter input_{camera_, gizmo_};
  mutable std::mutex cloud_mutex_;
  mutable std::mutex markers_mutex_;
  QVector<double> marker_positions_xyz_;
  std::optional<Eigen::Vector3f> pending_marker_;
  std::vector<SphereMarker> sphere_markers_;
  bool markers_dirty_{false};
  PointCloudData pending_cloud_;
  bool cloud_dirty_{false};
  std::uint64_t cloud_revision_{0};
  bool content_bounds_valid_{false};
  Eigen::Vector3f content_bounds_min_{0.f, 0.f, 0.f};
  Eigen::Vector3f content_bounds_max_{0.f, 0.f, 0.f};
  double blend_weight_{0.};
  double scalar_min_{0.};
  double scalar_max_{1.};
  bool interaction_enabled_{true};
  double marker_sphere_radius_{0.05};
};


}  // namespace psiviz
