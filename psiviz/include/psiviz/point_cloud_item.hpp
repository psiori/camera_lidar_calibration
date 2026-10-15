#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include <clc/linalg.hpp>

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

/// QML item that renders a colored point cloud with trackball navigation, picking, and a pose gizmo.
class PointCloudItem : public QQuickRhiItem {
  Q_OBJECT
  /// Registered in QML as `PointCloudScene` (see PointCloudViewerPane.qml).
  QML_NAMED_ELEMENT(PointCloudScene)

  /// 0 = color by scalar colormap only; 1 = full per-point RGB from AppController.
  Q_PROPERTY(double blendWeight READ blendWeight WRITE setBlendWeight NOTIFY blendWeightChanged)
  /// Scalar values below this map to the blue end of the JET LUT.
  Q_PROPERTY(double scalarMin READ scalarMin WRITE setScalarMin NOTIFY scalarRangeChanged)
  Q_PROPERTY(double scalarMax READ scalarMax WRITE setScalarMax NOTIFY scalarRangeChanged)
  /// Point sprite diameter in pixels.
  Q_PROPERTY(double pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
  /// Draw the camera–lidar pose gizmo when true.
  Q_PROPERTY(bool gizmoEnabled READ gizmoEnabled WRITE setGizmoEnabled NOTIFY gizmoEnabledChanged)
  /// Allow dragging gizmo axes/rings when true (still requires gizmoEnabled).
  Q_PROPERTY(bool gizmoManipulable READ gizmoManipulable WRITE setGizmoManipulable NOTIFY gizmoManipulableChanged)
  /// Allow orbit/pan/zoom when true.
  Q_PROPERTY(bool interactionEnabled READ interactionEnabled WRITE setInteractionEnabled NOTIFY interactionEnabledChanged)
  /// Allow Shift+click and programmatic picks when true.
  Q_PROPERTY(bool pickingEnabled READ pickingEnabled WRITE setPickingEnabled NOTIFY pickingEnabledChanged)
  /// World-space radius of correspondence / pick marker spheres.
  Q_PROPERTY(double markerSphereRadius READ markerSphereRadius WRITE setMarkerSphereRadius NOTIFY markerSphereRadiusChanged)

 public:
  explicit PointCloudItem(QQuickItem* parent = nullptr);

  /// Weight between scalar colormap (0) and per-point RGB (1).
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
  bool gizmoManipulable() const { return gizmo_manipulable_; }
  void setGizmoManipulable(bool m);
  bool interactionEnabled() const { return interaction_enabled_; }
  void setInteractionEnabled(bool e);
  bool pickingEnabled() const { return picking_enabled_; }
  void setPickingEnabled(bool e);
  [[nodiscard]] bool hasPointCloudContent() const;
  [[nodiscard]] std::uint64_t pointCloudPointCount() const;

  /// Replace full cloud buffers and mark GPU upload dirty.
  Q_INVOKABLE void setPointCloudData(const QVector<float>& positions, const QVector<float>& scalars,
                                     const QVector<float>& rgb);
  /// Updates scalar/RGB buffers only; keeps camera and positions unchanged.
  Q_INVOKABLE void updatePointCloudScalarsAndColors(const QVector<float>& scalars, const QVector<float>& rgb);
  /// Fit the camera to the current point cloud bounding box.
  Q_INVOKABLE void frameCameraToContent();
  /// Refresh near/far clip planes from the current orbit (use if points are clipped after zoom/pan).
  Q_INVOKABLE void recomputeClipPlanes();
  /// 4x4 pose matrix as 16 doubles in column-major order (for QML / AppController).
  Q_INVOKABLE QVariantList poseMatrix() const;
  Q_INVOKABLE void setPoseMatrix(const QVariantList& values);
  /// Navigation keys (`f`, `+`, `-`, `r`, `h`, `z`) for QML forwarding when the pointer is over the viewport.
  Q_INVOKABLE bool handleNavigationKey(int key, int modifiers);
  Q_INVOKABLE void bumpPointSize(double delta);
  /// Point-index pick at last pointer position (or viewport center) and animate focal point there (`f`).
  Q_INVOKABLE void flyToUnderCursor();
  /// Last known mouse position in item coordinates (for fly-to and deferred picks).
  Q_INVOKABLE void setPointerPosition(double x, double y);
  /// Replace pick markers with spheres at each (x,y,z) triple in `positions_xyz` (length must be a multiple of 3).
  Q_INVOKABLE void setSphereMarkerPositions(const QVector<double>& positions_xyz);
  /// Show a semi-transparent preview sphere before the correspondence is committed.
  Q_INVOKABLE void setPendingSphereMarker(double x, double y, double z);
  Q_INVOKABLE void clearPendingSphereMarker();
  Q_INVOKABLE void clearSphereMarkers();
  /// Same pick path as Shift+click at viewport pixel (x, y); emits `pointPicked` when complete.
  Q_INVOKABLE void requestPickAt(int x, int y);

  double markerSphereRadius() const { return marker_sphere_radius_; }
  void setMarkerSphereRadius(double radius);

 signals:
  /// NOTIFY for blendWeight Q_PROPERTY.
  void blendWeightChanged();
  /// NOTIFY for scalarMin and scalarMax.
  void scalarRangeChanged();
  void pointSizeChanged();
  void gizmoEnabledChanged();
  void gizmoManipulableChanged();
  void interactionEnabledChanged();
  void pickingEnabledChanged();
  void markerSphereRadiusChanged();
  /// Emitted after a successful GPU pick with world-space coordinates.
  void pointPicked(double x, double y, double z);
  /// Emitted when the pose gizmo transform changes.
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

  int pick_x_{0};                      /// Pick request X in item coordinates.
  int pick_y_{0};                      /// Pick request Y in item coordinates.
  bool do_pick_{false};                /// Renderer should run pick pass this frame.
  bool fly_to_{false};                 /// After pick, animate camera to hit point.
  bool pick_emit_signal_{false};       /// Emit pointPicked when pick completes.
  bool emit_pose_{false};              /// Emit poseChanged after gizmo drag.
  QPointF last_mouse_;                 /// Cached pointer for picks and fly-to.
  bool pointer_position_valid_{false};

  TrackballCamera camera_;
  PoseGizmo gizmo_;
  ViewerInputRouter input_{camera_, gizmo_};
  mutable std::mutex cloud_mutex_;     /// Protects pending_cloud_ from UI vs render thread.
  mutable std::mutex markers_mutex_;   /// Protects marker_positions_xyz_ and sphere_markers_.
  QVector<double> marker_positions_xyz_;  /// Flat x,y,z,... storage for committed markers.
  std::optional<clc::Vec3f> pending_marker_;
  std::vector<SphereMarker> sphere_markers_;
  bool markers_dirty_{false};
  PointCloudData pending_cloud_;
  bool cloud_dirty_{false};
  std::uint64_t cloud_revision_{0};    /// Bumped on each full cloud upload.
  bool content_bounds_valid_{false};
  clc::Vec3f content_bounds_min_{0.f, 0.f, 0.f};
  clc::Vec3f content_bounds_max_{0.f, 0.f, 0.f};
  double blend_weight_{0.};
  double scalar_min_{0.};
  double scalar_max_{1.};
  bool interaction_enabled_{true};
  bool picking_enabled_{true};
  bool gizmo_manipulable_{true};
  double marker_sphere_radius_{0.05};
};


}  // namespace psiviz
