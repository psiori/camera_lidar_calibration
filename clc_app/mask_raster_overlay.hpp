#pragma once

#include <QImage>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>
#include <QPointF>
#include <functional>
#include <vector>

/// Shared mask-editor painting helpers for range-image and camera-image mask items.
namespace clc_app::mask_overlay {

/// Darken excluded mask pixels (value 0) over `dest`.
void paintExcludedMaskOverlay(
  QPainter* painter,
  const QRectF& dest,
  int mask_width,
  int mask_height,
  const QByteArray& mask_data);

/// Draw polygon vertices and edges in display coordinates.
void paintPolygonOverlay(
  QPainter* painter,
  const std::vector<QPointF>& native_vertices,
  bool closed,
  const std::function<QPointF(const QPointF&)>& native_to_display,
  bool has_hover,
  bool hover_use_display,
  const QPointF& hover_native,
  const QPointF& hover_display);

}  // namespace clc_app::mask_overlay
