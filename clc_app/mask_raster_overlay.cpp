#include "mask_raster_overlay.hpp"

#include <QColor>
#include <QLineF>
#include <QPen>

namespace clc_app::mask_overlay {

void paintExcludedMaskOverlay(
  QPainter* painter,
  const QRectF& dest,
  int mask_width,
  int mask_height,
  const QByteArray& mask_data) {
  if (mask_width <= 0 || mask_height <= 0 || mask_data.size() != mask_width * mask_height) {
    return;
  }
  QImage overlay(mask_width, mask_height, QImage::Format_ARGB32);
  overlay.fill(Qt::transparent);
  const auto* bytes = reinterpret_cast<const uchar*>(mask_data.constData());
  for (int y = 0; y < mask_height; ++y) {
    for (int x = 0; x < mask_width; ++x) {
      if (bytes[y * mask_width + x] == 0) {
        overlay.setPixelColor(x, y, QColor(0, 0, 0, 255));
      }
    }
  }
  painter->drawImage(dest, overlay);
}

void paintPolygonOverlay(
  QPainter* painter,
  const std::vector<QPointF>& native_vertices,
  bool closed,
  const std::function<QPointF(const QPointF&)>& native_to_display,
  bool has_hover,
  bool hover_use_display,
  const QPointF& hover_native,
  const QPointF& hover_display) {
  if (native_vertices.empty()) {
    return;
  }
  QPen pen(QColor(255, 220, 0));
  pen.setWidthF(2.0);
  painter->setPen(pen);
  QPolygonF poly;
  for (const auto& v : native_vertices) {
    poly << native_to_display(v);
  }
  if (!closed && has_hover) {
    poly << (hover_use_display ? hover_display : native_to_display(hover_native));
    painter->drawPolyline(poly);
  } else if (closed) {
    painter->drawPolygon(poly);
  } else {
    painter->drawPolyline(poly);
  }
  painter->setBrush(QColor(255, 220, 0));
  for (const auto& v : native_vertices) {
    const QPointF d = native_to_display(v);
    painter->drawEllipse(d, 3, 3);
  }
  if (native_vertices.size() >= 3) {
    const QPointF first = native_to_display(native_vertices.front());
    painter->setPen(QPen(QColor(80, 255, 120), 2.0));
    painter->setBrush(QColor(80, 255, 120));
    painter->drawEllipse(first, 5, 5);
  }
}

}  // namespace clc_app::mask_overlay
