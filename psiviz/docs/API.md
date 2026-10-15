# psiviz API

## Threading and buffer upload

- **GUI thread:** `PointCloudItem` properties, input events, camera/gizmo state.
- **Worker threads:** may call `PointCloudItem::setPointCloudData()` with a snapshot; data is copied under a mutex and consumed in `QQuickRhiItemRenderer::synchronize()`.
- Do not call QRhi or OpenGL from worker threads.

## `psiviz::Colormap`

JET lookup table (`size()` entries, RGB float 0–1). Used by the point cloud fragment shader via min/max uniforms.

## `psiviz::TrackballCamera`

View navigation with PCL-style bindings. Methods: `rotate`, `pan`, `dolly`, `spin`, `reset`, `flyTo`, `viewMatrix`, `projectionMatrix`, `fitToBounds`, `recomputeClipPlanes`. Near/far clip planes track orbit distance (and the last fitted bounding sphere). Use `PointCloudItem::frameCameraToContent()` (or `H` / `Home`) to fit the camera to loaded points; use `recomputeClipPlanes()` (or `Z`) if points look clipped after zooming; data uploads do not reframe automatically.

## `psiviz::DepthPicker`

Static helpers for pick resolution (used by `PointCloudRenderer` / `PointCloudItem`):

- `unproject(depth01, x, y, width, height, device_pixel_ratio, clip_view_projection)` — NDC depth from the pick pass (alpha) back to world space; `clip_view_projection` must match the point shader MVP (`clip_corr * projection * view`).
- `pickNearestOnScreen(positions, point_count, x, y, width, height, device_pixel_ratio, clip_view_projection, max_pixel_radius)` — CPU fallback when GPU depth pick misses; returns the nearest cloud point in screen space within `max_pixel_radius`.

## `psiviz::PoseGizmo`

Holds `clc::Mat4f` pose (column-major 4×4, same layout as Eigen/OpenGL). `hitTest`, `drag`, `isManipulating()`. Rendered by `PointCloudRenderer` after the point cloud.

## `psiviz::ViewerInputRouter`

Priority: gizmo drag > Shift+left-drag pan / Shift+click pick > camera. Returns `InputAction` for the item to apply.

## `psiviz::PointCloudItem` (QML: `PointCloudScene`)

| Property | Description |
|----------|-------------|
| `blendWeight` | 0 = scalar colormap only, 1 = RGB only |
| `scalarMin`, `scalarMax` | JET range |
| `pointSize` | Screen-space point diameter |
| `gizmoEnabled` | Draw the camera pose gizmo at `poseMatrix` |
| `gizmoManipulable` | When false, gizmo is display-only (no hit-test or drag); trackball and Shift+pick unchanged |
| `interactionEnabled` | Camera/gizmo navigation |
| `pickingEnabled` | Shift+click point pick (when interaction is enabled) |

| Signal | Description |
|--------|-------------|
| `pointPicked(x, y, z)` | Shift+click pick; world-space coordinates (depth pick + optional CPU fallback) |

### Scene shapes (non-pickable)

Rudimentary meshes (UV spheres) render in the main pass only; they are **not** drawn in the depth pick pass.

| Method | Description |
|--------|-------------|
| `setSphereMarkerPositions(xyz…)` | Flat `x,y,z` triples; 0.05 m orange markers at 50% opacity |
| `setPendingSphereMarker(x,y,z)` | Same style for an in-progress pick |
| `clearPendingSphereMarker()` | Remove pending marker |
| `clearSphereMarkers()` | Remove all markers |
| `markerSphereRadius` | World radius in scene units (`0` = default 0.05) |
| `poseChanged()` | Gizmo moved the pose matrix |

`poseMatrix()` / `setPoseMatrix()` expose the 4×4 pose as 16 doubles (column-major).
