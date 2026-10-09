#pragma once

#include <cstdint>
#include <mutex>

#include <QPointF>
#include <QQuickRhiItem>
#include <QtQml/qqmlregistration.h>

#include "psiviz/point_cloud_buffers.hpp"
#include "psiviz/pose_gizmo.hpp"
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

  Q_INVOKABLE void setPointCloudData(const QVector<float>& positions, const QVector<float>& scalars,
                                     const QVector<float>& rgb);
  Q_INVOKABLE QVariantList poseMatrix() const;
  Q_INVOKABLE void setPoseMatrix(const QVariantList& values);

 signals:
  void blendWeightChanged();
  void scalarRangeChanged();
  void pointSizeChanged();
  void gizmoEnabledChanged();
  void interactionEnabledChanged();
  void pointPicked(double x, double y, double z);
  void poseChanged();

 protected:
  QQuickRhiItemRenderer* createRenderer() override;
  bool event(QEvent* event) override;

 private:
  friend class PointCloudRendererItem;
  void updateViewport();

  int pick_x_{0};
  int pick_y_{0};
  bool do_pick_{false};
  bool fly_to_{false};
  bool emit_pose_{false};
  QPointF last_mouse_;

  TrackballCamera camera_;
  PoseGizmo gizmo_;
  ViewerInputRouter input_{camera_, gizmo_};
  std::mutex cloud_mutex_;
  PointCloudData pending_cloud_;
  bool cloud_dirty_{false};
  std::uint64_t cloud_revision_{0};
  double blend_weight_{0.};
  double scalar_min_{0.};
  double scalar_max_{1.};
  bool interaction_enabled_{true};
};

}  // namespace psiviz
