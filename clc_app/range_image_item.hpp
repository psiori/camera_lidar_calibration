#pragma once

#include <QImage>
#include <QPointF>
#include <QVariant>
#include <QQuickPaintedItem>
#include <QtQml/qqmlregistration.h>
#include <vector>

class RangeImageItem : public QQuickPaintedItem {
  Q_OBJECT
  QML_NAMED_ELEMENT(RangeImageItem)
  Q_PROPERTY(QImage image READ image WRITE setImage NOTIFY imageChanged)
  Q_PROPERTY(QByteArray maskData READ maskData WRITE setMaskData NOTIFY maskDataChanged)
  Q_PROPERTY(int maskWidth READ maskWidth WRITE setMaskWidth NOTIFY maskDataChanged)
  Q_PROPERTY(int maskHeight READ maskHeight WRITE setMaskHeight NOTIFY maskDataChanged)
  Q_PROPERTY(bool drawing READ drawing NOTIFY drawingChanged)
  Q_PROPERTY(bool fillBounds READ fillBounds WRITE setFillBounds NOTIFY fillBoundsChanged)
  Q_PROPERTY(QVariantList closedPolygonOverlay READ closedPolygonOverlay WRITE setClosedPolygonOverlay
                 NOTIFY closedPolygonOverlayChanged)
  Q_PROPERTY(QObject* maskController READ maskController WRITE setMaskController NOTIFY maskControllerChanged)

 public:
  explicit RangeImageItem(QQuickItem* parent = nullptr);

  QImage image() const { return image_; }
  void setImage(const QImage& image);
  QByteArray maskData() const { return mask_data_; }
  void setMaskData(const QByteArray& data);
  int maskWidth() const { return mask_width_; }
  void setMaskWidth(int w);
  int maskHeight() const { return mask_height_; }
  void setMaskHeight(int h);
  bool drawing() const { return drawing_; }
  bool fillBounds() const { return fill_bounds_; }
  void setFillBounds(bool fill);
  QVariantList closedPolygonOverlay() const { return closed_polygon_overlay_; }
  void setClosedPolygonOverlay(const QVariantList& vertices);
  QObject* maskController() const { return mask_controller_; }
  void setMaskController(QObject* controller);

  Q_INVOKABLE void undoVertex();
  Q_INVOKABLE void closePolygon();
  Q_INVOKABLE void cancelPolygon();
  Q_INVOKABLE void clearPendingPolygon();

 signals:
  void imageChanged();
  void maskDataChanged();
  void drawingChanged();
  void fillBoundsChanged();
  void closedPolygonOverlayChanged();
  void maskControllerChanged();
  void polygonCommitted(QVariantList nativeVertices);

 protected:
  void paint(QPainter* painter) override;
  void mousePressEvent(QMouseEvent* event) override;
  void hoverMoveEvent(QHoverEvent* event) override;
  void geometryChange(const QRectF& newGeometry, const QRectF& oldGeometry) override;

 private:
  int nativeWidth() const;
  int nativeHeight() const;
  qreal displayYScale() const;
  qreal displayScale() const;
  QRectF imageDestRect() const;
  QPointF displayToNative(const QPointF& display) const;
  QPointF nativeToDisplay(const QPointF& native) const;
  QRectF interactionMarginRect() const;
  bool interactionRectContains(const QPointF& display) const;
  QPointF nativeVertexForClick(const QPointF& display) const;
  QPointF hoverPointForDisplay(const QPointF& display) const;
  void updateHover(const QPointF& display);
  void addVertexAtDisplay(const QPointF& display);
  void setDrawing(bool drawing);
  void updateImplicitHeight();

  QImage image_;
  QByteArray mask_data_;
  int mask_width_ = 0;
  int mask_height_ = 0;
  bool drawing_ = false;
  bool fill_bounds_ = false;
  std::vector<QPointF> current_native_vertices_;
  std::vector<QPointF> pending_closed_native_vertices_;
  QVariantList closed_polygon_overlay_;
  QObject* mask_controller_ = nullptr;
  QPointF hover_native_;
  QPointF hover_display_;
  bool hover_use_display_ = false;
  bool has_hover_ = false;
};
