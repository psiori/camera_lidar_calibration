#pragma once

#include <QImage>
#include <QPointF>
#include <QRectF>
#include <QVariant>
#include <QQuickPaintedItem>
#include <QtQml/qqmlregistration.h>
#include <vector>

/// QML item that paints a range-image preview and captures polygon vertices for the global mask editor.
class RangeImageItem : public QQuickPaintedItem {
  Q_OBJECT
  QML_NAMED_ELEMENT(RangeImageItem)

  /// Colored range image from AppController.rangeImage.
  Q_PROPERTY(QImage image READ image WRITE setImage NOTIFY imageChanged)
  /// Raw mask raster (same encoding as AppController.maskPreview).
  Q_PROPERTY(QByteArray maskData READ maskData WRITE setMaskData NOTIFY maskDataChanged)
  Q_PROPERTY(int maskWidth READ maskWidth WRITE setMaskWidth NOTIFY maskDataChanged)
  Q_PROPERTY(int maskHeight READ maskHeight WRITE setMaskHeight NOTIFY maskDataChanged)
  /// Structured scan width for mapping display clicks to native pixels.
  Q_PROPERTY(int rangeNativeWidth READ rangeNativeWidth WRITE setRangeNativeWidth NOTIFY rangeNativeSizeChanged)
  Q_PROPERTY(int rangeNativeHeight READ rangeNativeHeight WRITE setRangeNativeHeight NOTIFY rangeNativeSizeChanged)
  /// True while placing vertices for an open polygon.
  Q_PROPERTY(bool drawing READ drawing NOTIFY drawingChanged)
  /// Scale image to fill item height (mask editor layout).
  Q_PROPERTY(bool fillBounds READ fillBounds WRITE setFillBounds NOTIFY fillBoundsChanged)
  /// Closed polygon drawn in display coordinates before keep/remove.
  Q_PROPERTY(QVariantList closedPolygonOverlay READ closedPolygonOverlay WRITE setClosedPolygonOverlay
                 NOTIFY closedPolygonOverlayChanged)
  /// AppController (or test stub); receives polygonCommitted.
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
  int rangeNativeWidth() const { return range_native_width_; }
  void setRangeNativeWidth(int w);
  int rangeNativeHeight() const { return range_native_height_; }
  void setRangeNativeHeight(int h);
  /// True while the user is placing vertices for an open polygon.
  bool drawing() const { return drawing_; }
  /// When true, scale the image to fill the item height (mask editor layout).
  bool fillBounds() const { return fill_bounds_; }
  void setFillBounds(bool fill);
  /// Vertices of a closed polygon drawn in display space (for overlay preview).
  QVariantList closedPolygonOverlay() const { return closed_polygon_overlay_; }
  void setClosedPolygonOverlay(const QVariantList& vertices);
  /// AppController (or test double) that receives committed polygons.
  QObject* maskController() const { return mask_controller_; }
  void setMaskController(QObject* controller);

  // QML keyboard / toolbar actions
  /// Remove the last placed vertex of the open polygon.
  Q_INVOKABLE void undoVertex();
  /// Finish the polygon and wait for keep/remove decision from the controller.
  Q_INVOKABLE void closePolygon();
  /// Discard the open polygon without committing.
  Q_INVOKABLE void cancelPolygon();
  /// Clear a closed polygon waiting for keep/remove.
  Q_INVOKABLE void clearPendingPolygon();

#ifdef CLC_APP_TESTING
  Q_INVOKABLE void clickAtDisplayPosition(qreal x, qreal y);
  Q_INVOKABLE int closedPolygonVertexCount() const;
  Q_INVOKABLE bool containsPolygonOutlinePixels() const;
  Q_INVOKABLE QRectF imageDestinationRectForTest() const;
  Q_INVOKABLE QRectF interactionMarginRectForTest() const;
#endif

 signals:
  void imageChanged();
  void maskDataChanged();
  /// NOTIFY for rangeNativeWidth and rangeNativeHeight.
  void rangeNativeSizeChanged();
  void drawingChanged();
  void fillBoundsChanged();
  void closedPolygonOverlayChanged();
  void maskControllerChanged();
  /// Native range-image pixel coordinates; AppController connects this to commitMaskPolygon.
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
  bool hasClosedPolygonAwaitingDecision() const;
  std::vector<QPointF> closedPolygonNativeVertices() const;
  QPointF maskNativeToRangeNative(const QPointF& p) const;

  QImage image_;
  QByteArray mask_data_;
  int mask_width_ = 0;
  int mask_height_ = 0;
  int range_native_width_ = 0;
  int range_native_height_ = 0;
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
