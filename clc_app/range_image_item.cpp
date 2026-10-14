#include "range_image_item.hpp"

#include "app_controller.hpp"

#include <limits>

#include <QPainter>
#include <QMouseEvent>
#include <QtMath>

#ifdef CLC_APP_TESTING
#include <QEventLoop>
#include <QQuickItemGrabResult>
#endif

namespace {

constexpr qreal kEdgeClickMargin = 48.0;
constexpr qreal kCloseVertexPixels = 10.0;

QRectF nativeImageRect(int w, int h) {
  if (w <= 0 || h <= 0) {
    return {};
  }
  return QRectF(0, 0, static_cast<qreal>(w - 1), static_cast<qreal>(h - 1));
}

bool nativeInside(const QPointF& p, int w, int h) {
  return p.x() >= 0 && p.y() >= 0 && p.x() < w && p.y() < h;
}

QPointF clampNativeToImage(const QPointF& p, int w, int h) {
  return {qBound(0.0, p.x(), static_cast<qreal>(w - 1)), qBound(0.0, p.y(), static_cast<qreal>(h - 1))};
}

QPointF nearestNativeOnImageEdge(const QPointF& p, int w, int h) {
  const QRectF r = nativeImageRect(w, h);
  if (r.isEmpty()) {
    return {};
  }
  if (r.contains(p)) {
    return p;
  }
  const qreal left = r.left();
  const qreal right = r.right();
  const qreal top = r.top();
  const qreal bottom = r.bottom();
  const qreal dx_left = std::abs(p.x() - left);
  const qreal dx_right = std::abs(p.x() - right);
  const qreal dy_top = std::abs(p.y() - top);
  const qreal dy_bottom = std::abs(p.y() - bottom);
  const qreal m = std::min({dx_left, dx_right, dy_top, dy_bottom});
  if (m == dx_left) {
    return {left, qBound(top, p.y(), bottom)};
  }
  if (m == dx_right) {
    return {right, qBound(top, p.y(), bottom)};
  }
  if (m == dy_top) {
    return {qBound(left, p.x(), right), top};
  }
  return {qBound(left, p.x(), right), bottom};
}

// Point on the native image border where the ray a -> b (infinite) first hits the border, closest to b.
QPointF rayToImageEdge(const QPointF& a, const QPointF& b, int w, int h) {
  const QRectF r = nativeImageRect(w, h);
  if (r.isEmpty()) {
    return {};
  }
  if (r.contains(b)) {
    return clampNativeToImage(b, w, h);
  }

  const QPointF dir = b - a;
  const qreal len = std::hypot(dir.x(), dir.y());
  if (len < 1e-9) {
    return nearestNativeOnImageEdge(b, w, h);
  }

  qreal t_best = std::numeric_limits<qreal>::infinity();
  const auto try_t = [&](qreal t) {
    if (t <= 1e-9) {
      return;
    }
    const QPointF p = a + dir * t;
    const bool on_border = p.x() >= r.left() && p.x() <= r.right() && p.y() >= r.top() && p.y() <= r.bottom() &&
                           (qFuzzyCompare(p.x(), r.left()) || qFuzzyCompare(p.x(), r.right()) ||
                            qFuzzyCompare(p.y(), r.top()) || qFuzzyCompare(p.y(), r.bottom()));
    if (on_border) {
      t_best = std::min(t_best, t);
    }
  };

  if (!qFuzzyIsNull(dir.x())) {
    try_t((r.left() - a.x()) / dir.x());
    try_t((r.right() - a.x()) / dir.x());
  }
  if (!qFuzzyIsNull(dir.y())) {
    try_t((r.top() - a.y()) / dir.y());
    try_t((r.bottom() - a.y()) / dir.y());
  }

  if (std::isfinite(t_best)) {
    return clampNativeToImage(a + dir * t_best, w, h);
  }
  return nearestNativeOnImageEdge(b, w, h);
}

}  // namespace

RangeImageItem::RangeImageItem(QQuickItem* parent) : QQuickPaintedItem(parent) {
  setAcceptedMouseButtons(Qt::LeftButton);
  setAcceptHoverEvents(true);
  setAntialiasing(true);
}

/// RangeImageItem::setImage — set Image.
void RangeImageItem::setImage(const QImage& image) {
  if (image_ == image) {
    return;
  }
  image_ = image;
  cancelPolygon();
  updateImplicitHeight();
  update();
  emit imageChanged();
}

/// RangeImageItem::setMaskData — set Mask Data.
void RangeImageItem::setMaskData(const QByteArray& data) {
  if (mask_data_ == data) {
    return;
  }
  mask_data_ = data;
  update();
  emit maskDataChanged();
}

/// RangeImageItem::setMaskWidth — set Mask Width.
void RangeImageItem::setMaskWidth(int w) {
  if (mask_width_ == w) {
    return;
  }
  mask_width_ = w;
  update();
  emit maskDataChanged();
}

/// RangeImageItem::setMaskHeight — set Mask Height.
void RangeImageItem::setMaskHeight(int h) {
  if (mask_height_ == h) {
    return;
  }
  mask_height_ = h;
  update();
  emit maskDataChanged();
}

/// RangeImageItem::setRangeNativeWidth — set Range Native Width.
void RangeImageItem::setRangeNativeWidth(int w) {
  if (range_native_width_ == w) {
    return;
  }
  range_native_width_ = w;
  update();
  emit rangeNativeSizeChanged();
}

/// RangeImageItem::setRangeNativeHeight — set Range Native Height.
void RangeImageItem::setRangeNativeHeight(int h) {
  if (range_native_height_ == h) {
    return;
  }
  range_native_height_ = h;
  update();
  emit rangeNativeSizeChanged();
}

/// RangeImageItem::setDrawing — set Drawing.
void RangeImageItem::setDrawing(bool drawing) {
  if (drawing_ == drawing) {
    return;
  }
  drawing_ = drawing;
  emit drawingChanged();
}

/// RangeImageItem::setFillBounds — set Fill Bounds.
void RangeImageItem::setFillBounds(bool fill) {
  if (fill_bounds_ == fill) {
    return;
  }
  fill_bounds_ = fill;
  updateImplicitHeight();
  update();
  emit fillBoundsChanged();
}

/// RangeImageItem::setClosedPolygonOverlay — set Closed Polygon Overlay.
void RangeImageItem::setClosedPolygonOverlay(const QVariantList& vertices) {
  if (closed_polygon_overlay_ == vertices) {
    return;
  }
  closed_polygon_overlay_ = vertices;
  update();
  emit closedPolygonOverlayChanged();
}

/// RangeImageItem::setMaskController — set Mask Controller.
void RangeImageItem::setMaskController(QObject* controller) {
  if (mask_controller_ == controller) {
    return;
  }
  if (mask_controller_) {
    disconnect(mask_controller_, nullptr, this, nullptr);
  }
  mask_controller_ = controller;
  if (auto* app = qobject_cast<AppController*>(mask_controller_)) {
    connect(app, &AppController::maskPolygonPendingChanged, this, [this] { update(); });
    connect(app, &AppController::maskChanged, this, [this] { update(); });
  }
  emit maskControllerChanged();
}

/// RangeImageItem::imageDestRect — image Dest Rect.
QRectF RangeImageItem::imageDestRect() const {
  if (image_.isNull() || width() <= 0) {
    return {};
  }
  const qreal scale = displayScale();
  const qreal iw = image_.width() * scale;
  const qreal ih = image_.height() * scale;
  qreal x0 = kEdgeClickMargin;
  qreal y0 = kEdgeClickMargin;
  if (fill_bounds_ && height() > 0) {
    x0 = (width() - iw) * 0.5;
    y0 = (height() - ih) * 0.5;
  }
  return {x0, y0, iw, ih};
}

/// RangeImageItem::interactionMarginRect — interaction Margin Rect.
QRectF RangeImageItem::interactionMarginRect() const {
  return imageDestRect().adjusted(-kEdgeClickMargin, -kEdgeClickMargin, kEdgeClickMargin, kEdgeClickMargin);
}

/// RangeImageItem::interactionRectContains — interaction Rect Contains.
bool RangeImageItem::interactionRectContains(const QPointF& display) const {
  return interactionMarginRect().contains(display);
}

/// RangeImageItem::updateImplicitHeight — update Implicit Height.
void RangeImageItem::updateImplicitHeight() {
  if (fill_bounds_) {
    setImplicitHeight(-1);
    return;
  }
  if (image_.isNull() || width() <= 0) {
    setImplicitHeight(120);
    return;
  }
  const QRectF dest = imageDestRect();
  setImplicitHeight(dest.height() + 2.0 * kEdgeClickMargin);
}

/// RangeImageItem::nativeWidth — native Width.
int RangeImageItem::nativeWidth() const {
  if (range_native_width_ > 0) {
    return range_native_width_;
  }
  if (!image_.isNull() && image_.width() > 0) {
    return image_.width();
  }
  return mask_width_ > 0 ? mask_width_ : 0;
}

/// RangeImageItem::nativeHeight — native Height.
int RangeImageItem::nativeHeight() const {
  if (range_native_height_ > 0) {
    return range_native_height_;
  }
  if (!image_.isNull() && image_.width() > 0 && image_.height() > 0) {
    const qreal y_scale = displayYScale();
    if (y_scale > 0) {
      return std::max(1, static_cast<int>(std::lround(static_cast<qreal>(image_.height()) / y_scale)));
    }
  }
  return mask_height_ > 0 ? mask_height_ : 0;
}

/// RangeImageItem::displayYScale — display YScale.
qreal RangeImageItem::displayYScale() const {
  const int unstretched_h = range_native_height_ > 0 ? range_native_height_
                                                     : (mask_height_ > 0 ? mask_height_ : 0);
  if (unstretched_h <= 0 || image_.height() <= 0) {
    return 1.0;
  }
  return static_cast<qreal>(image_.height()) / static_cast<qreal>(unstretched_h);
}

/// RangeImageItem::displayScale — display Scale.
qreal RangeImageItem::displayScale() const {
  if (image_.isNull() || image_.width() <= 0 || image_.height() <= 0 || width() <= 0) {
    return 1.0;
  }
  const qreal avail_w = width() - 2.0 * kEdgeClickMargin;
  if (avail_w <= 0) {
    return 1.0;
  }
  const qreal scale_w = avail_w / static_cast<qreal>(image_.width());
  if (!fill_bounds_ || height() <= 0) {
    return scale_w;
  }
  const qreal avail_h = height() - 2.0 * kEdgeClickMargin;
  if (avail_h <= 0) {
    return scale_w;
  }
  const qreal scale_h = avail_h / static_cast<qreal>(image_.height());
  return std::min(scale_w, scale_h);
}

/// RangeImageItem::displayToNative — display To Native.
QPointF RangeImageItem::displayToNative(const QPointF& display) const {
  const QRectF dest = imageDestRect();
  const qreal scale = displayScale();
  const qreal y_scale = displayYScale();
  if (scale <= 0 || dest.isEmpty()) {
    return {};
  }
  const QPointF local = display - dest.topLeft();
  return {local.x() / scale, local.y() / (scale * y_scale)};
}

/// RangeImageItem::nativeToDisplay — native To Display.
QPointF RangeImageItem::nativeToDisplay(const QPointF& native) const {
  const QRectF dest = imageDestRect();
  const qreal scale = displayScale();
  const qreal y_scale = displayYScale();
  return dest.topLeft() + QPointF(native.x() * scale, native.y() * scale * y_scale);
}

/// RangeImageItem::nativeVertexForClick — native Vertex For Click.
QPointF RangeImageItem::nativeVertexForClick(const QPointF& display) const {
  const int w = nativeWidth();
  const int h = nativeHeight();
  const QRectF dest = imageDestRect();
  const QPointF native = displayToNative(display);

  if (!current_native_vertices_.empty()) {
    if (dest.contains(display)) {
      return clampNativeToImage(native, w, h);
    }
    return rayToImageEdge(current_native_vertices_.back(), native, w, h);
  }
  if (dest.contains(display)) {
    return clampNativeToImage(native, w, h);
  }
  return nearestNativeOnImageEdge(native, w, h);
}

/// RangeImageItem::hoverPointForDisplay — hover Point For Display.
QPointF RangeImageItem::hoverPointForDisplay(const QPointF& display) const {
  const int w = nativeWidth();
  const int h = nativeHeight();
  const QRectF dest = imageDestRect();
  const QPointF native = displayToNative(display);

  if (dest.contains(display)) {
    return clampNativeToImage(native, w, h);
  }
  if (!current_native_vertices_.empty()) {
    return rayToImageEdge(current_native_vertices_.back(), native, w, h);
  }
  return nearestNativeOnImageEdge(native, w, h);
}

/// RangeImageItem::updateHover — update Hover.
void RangeImageItem::updateHover(const QPointF& display) {
  if (!drawing_ || image_.isNull()) {
    has_hover_ = false;
    return;
  }
  if (!interactionRectContains(display)) {
    has_hover_ = false;
    update();
    return;
  }

  const QRectF dest = imageDestRect();
  if (dest.contains(display)) {
    hover_native_ = hoverPointForDisplay(display);
    hover_use_display_ = false;
  } else {
    hover_display_ = display;
    hover_use_display_ = true;
  }
  has_hover_ = true;
  update();
}

/// RangeImageItem::undoVertex — undo Vertex.
void RangeImageItem::undoVertex() {
  if (current_native_vertices_.empty()) {
    return;
  }
  current_native_vertices_.pop_back();
  if (current_native_vertices_.empty()) {
    setDrawing(false);
    has_hover_ = false;
  }
  update();
}

/// RangeImageItem::closePolygon — close Polygon.
void RangeImageItem::closePolygon() {
  if (current_native_vertices_.size() < 3) {
    return;
  }
  QVariantList verts;
  for (const auto& p : current_native_vertices_) {
    verts.append(QVariant::fromValue(p));
  }
  pending_closed_native_vertices_ = current_native_vertices_;
  current_native_vertices_.clear();
  setDrawing(false);
  has_hover_ = false;
  update();
  if (auto* controller = qobject_cast<AppController*>(mask_controller_)) {
    controller->commitMaskPolygonNative(pending_closed_native_vertices_);
  }
  emit polygonCommitted(verts);
}

/// RangeImageItem::cancelPolygon — cancel Polygon.
void RangeImageItem::cancelPolygon() {
  current_native_vertices_.clear();
  setDrawing(false);
  has_hover_ = false;
  update();
}

/// RangeImageItem::clearPendingPolygon — clear Pending Polygon.
void RangeImageItem::clearPendingPolygon() {
  if (pending_closed_native_vertices_.empty() && closed_polygon_overlay_.empty()) {
    return;
  }
  pending_closed_native_vertices_.clear();
  closed_polygon_overlay_.clear();
  update();
}

/// RangeImageItem::hasClosedPolygonAwaitingDecision — has Closed Polygon Awaiting Decision.
bool RangeImageItem::hasClosedPolygonAwaitingDecision() const {
  if (auto* controller = qobject_cast<AppController*>(mask_controller_)) {
    if (controller->maskPolygonPending()) {
      return true;
    }
  }
  return !pending_closed_native_vertices_.empty();
}

/// RangeImageItem::maskNativeToRangeNative — mask Native To Range Native.
QPointF RangeImageItem::maskNativeToRangeNative(const QPointF& p) const {
  const int rw = nativeWidth();
  const int rh = nativeHeight();
  const int mw = mask_width_;
  const int mh = mask_height_;
  if (rw <= 0 || rh <= 0 || mw <= 0 || mh <= 0 || (mw == rw && mh == rh)) {
    return p;
  }
  return {p.x() * static_cast<qreal>(rw) / static_cast<qreal>(mw),
          p.y() * static_cast<qreal>(rh) / static_cast<qreal>(mh)};
}

/// RangeImageItem::closedPolygonNativeVertices — closed Polygon Native Vertices.
std::vector<QPointF> RangeImageItem::closedPolygonNativeVertices() const {
  if (auto* controller = qobject_cast<AppController*>(mask_controller_)) {
    const std::vector<QPointF> from_controller = controller->pendingMaskPolygonNativePoints();
    if (!from_controller.empty()) {
      std::vector<QPointF> range_space;
      range_space.reserve(from_controller.size());
      for (const QPointF& p : from_controller) {
        range_space.push_back(maskNativeToRangeNative(p));
      }
      return range_space;
    }
  }
  if (!pending_closed_native_vertices_.empty()) {
    return pending_closed_native_vertices_;
  }
  return {};
}

/// RangeImageItem::addVertexAtDisplay — add Vertex At Display.
void RangeImageItem::addVertexAtDisplay(const QPointF& display) {
  if (image_.isNull() || !interactionRectContains(display)) {
    return;
  }
  if (hasClosedPolygonAwaitingDecision()) {
    return;
  }
  const QPointF first_display = current_native_vertices_.empty()
                                  ? QPointF()
                                  : nativeToDisplay(current_native_vertices_.front());
  if (current_native_vertices_.size() >= 3 && !first_display.isNull() &&
      QLineF(display, first_display).length() <= kCloseVertexPixels) {
    closePolygon();
    return;
  }

  const QPointF native = nativeVertexForClick(display);
  if (!current_native_vertices_.empty()) {
    const QPointF& prev = current_native_vertices_.back();
    if (QLineF(prev, native).length() < 0.25) {
      return;
    }
  }

  current_native_vertices_.push_back(native);
  setDrawing(true);
  update();
}

/// RangeImageItem::paint — paint.
void RangeImageItem::paint(QPainter* painter) {
  painter->fillRect(boundingRect(), QColor(24, 24, 26));
  if (image_.isNull()) {
    painter->setPen(Qt::white);
    painter->drawText(boundingRect(), Qt::AlignCenter, QStringLiteral("Load PCDs to show range image"));
    return;
  }

  const QRectF margin_rect = interactionMarginRect();
  const QRectF dest = imageDestRect();
  painter->fillRect(margin_rect, QColor(42, 44, 48));
  painter->setPen(QPen(QColor(90, 95, 105), 1.0));
  painter->drawRect(margin_rect);
  painter->drawImage(dest, image_);

  if (mask_width_ > 0 && mask_height_ > 0 && mask_data_.size() == mask_width_ * mask_height_) {
    QImage overlay(mask_width_, mask_height_, QImage::Format_ARGB32);
    overlay.fill(Qt::transparent);
    const auto* bytes = reinterpret_cast<const uchar*>(mask_data_.constData());
    for (int y = 0; y < mask_height_; ++y) {
      for (int x = 0; x < mask_width_; ++x) {
        if (bytes[y * mask_width_ + x] == 0) {
          overlay.setPixelColor(x, y, QColor(0, 0, 0, 255));
        }
      }
    }
    painter->drawImage(dest, overlay);
  }

  painter->setPen(QPen(QColor(120, 130, 150), 1.5));
  painter->setBrush(Qt::NoBrush);
  painter->drawRect(dest);

  const auto draw_polygon_overlay = [&](const std::vector<QPointF>& vertices, bool closed) {
    if (vertices.empty()) {
      return;
    }
    QPen pen(QColor(255, 220, 0));
    pen.setWidthF(2.0);
    painter->setPen(pen);
    QPolygonF poly;
    for (const auto& v : vertices) {
      poly << nativeToDisplay(v);
    }
    if (!closed && has_hover_) {
      poly << (hover_use_display_ ? hover_display_ : nativeToDisplay(hover_native_));
      painter->drawPolyline(poly);
    } else if (closed) {
      painter->drawPolygon(poly);
    } else {
      painter->drawPolyline(poly);
    }
    painter->setBrush(QColor(255, 220, 0));
    for (const auto& v : vertices) {
      const QPointF d = nativeToDisplay(v);
      painter->drawEllipse(d, 3, 3);
    }
    if (vertices.size() >= 3) {
      const QPointF first = nativeToDisplay(vertices.front());
      painter->setPen(QPen(QColor(80, 255, 120), 2.0));
      painter->setBrush(QColor(80, 255, 120));
      painter->drawEllipse(first, 5, 5);
    }
  };

  const std::vector<QPointF> closed_native = closedPolygonNativeVertices();
  if (!closed_native.empty()) {
    draw_polygon_overlay(closed_native, true);
  }
  if (!current_native_vertices_.empty()) {
    draw_polygon_overlay(current_native_vertices_, false);
  }
}

/// RangeImageItem::mousePressEvent — mouse Press Event.
void RangeImageItem::mousePressEvent(QMouseEvent* event) {
  if (image_.isNull() || event->button() != Qt::LeftButton) {
    QQuickPaintedItem::mousePressEvent(event);
    return;
  }
  if (event->flags().testFlag(Qt::MouseEventCreatedDoubleClick)) {
    if (current_native_vertices_.size() >= 3) {
      closePolygon();
    }
    event->accept();
    return;
  }
  addVertexAtDisplay(event->position());
  event->accept();
}

/// RangeImageItem::geometryChange — geometry Change.
void RangeImageItem::geometryChange(const QRectF& newGeometry, const QRectF& oldGeometry) {
  QQuickPaintedItem::geometryChange(newGeometry, oldGeometry);
  if (fill_bounds_) {
    if (!qFuzzyCompare(newGeometry.width(), oldGeometry.width()) ||
        !qFuzzyCompare(newGeometry.height(), oldGeometry.height())) {
      update();
    }
    return;
  }
  if (!qFuzzyCompare(newGeometry.width(), oldGeometry.width())) {
    updateImplicitHeight();
  }
}

/// RangeImageItem::hoverMoveEvent — hover Move Event.
void RangeImageItem::hoverMoveEvent(QHoverEvent* event) {
  updateHover(event->position());
  event->accept();
}

#ifdef CLC_APP_TESTING
/// RangeImageItem::clickAtDisplayPosition — click At Display Position.
void RangeImageItem::clickAtDisplayPosition(qreal x, qreal y) {
  addVertexAtDisplay(QPointF(x, y));
}

/// RangeImageItem::closedPolygonVertexCount — closed Polygon Vertex Count.
int RangeImageItem::closedPolygonVertexCount() const {
  return static_cast<int>(closedPolygonNativeVertices().size());
}

/// RangeImageItem::containsPolygonOutlinePixels — contains Polygon Outline Pixels.
bool RangeImageItem::containsPolygonOutlinePixels() const {
  auto* self = const_cast<RangeImageItem*>(this);
  const QSharedPointer<QQuickItemGrabResult> grab = self->grabToImage();
  if (!grab) {
    return false;
  }
  QEventLoop loop;
  QObject::connect(grab.data(), &QQuickItemGrabResult::ready, &loop, &QEventLoop::quit);
  loop.exec();
  const QImage image = grab->image();
  if (image.isNull()) {
    return false;
  }
  constexpr int kTol = 48;
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const QRgb px = image.pixel(x, y);
      const int r = qRed(px);
      const int g = qGreen(px);
      const int b = qBlue(px);
      if (std::abs(r - 255) <= kTol && std::abs(g - 220) <= kTol && std::abs(b - 0) <= kTol) {
        return true;
      }
      if (std::abs(r - 80) <= kTol && std::abs(g - 255) <= kTol && std::abs(b - 120) <= kTol) {
        return true;
      }
    }
  }
  return false;
}

/// RangeImageItem::imageDestinationRectForTest — image Destination Rect For Test.
QRectF RangeImageItem::imageDestinationRectForTest() const {
  return imageDestRect();
}

/// RangeImageItem::interactionMarginRectForTest — interaction Margin Rect For Test.
QRectF RangeImageItem::interactionMarginRectForTest() const {
  return interactionMarginRect();
}
#endif
