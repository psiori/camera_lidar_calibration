# psiviz API

## Threading and buffer upload

- **GUI thread:** `PointCloudItem` properties, input events, camera/gizmo state.
- **Worker threads:** may call `PointCloudItem::setPointCloudData()` with a snapshot; data is copied under a mutex and consumed in `QQuickRhiItemRenderer::synchronize()`.
- Do not call QRhi or OpenGL from worker threads.

## `psiviz::Colormap`

JET lookup table (`size()` entries, RGB float 0–1). Used by the point cloud fragment shader via min/max uniforms.

## `psiviz::TrackballCamera`

View navigation with PCL-style bindings. Methods: `rotate`, `pan`, `dolly`, `spin`, `reset`, `flyTo`, `viewMatrix`, `projectionMatrix`, `fitToBounds`. Use `PointCloudItem::frameCameraToContent()` (or `H` / `Home`) to fit the camera to loaded points; data uploads do not reframe automatically.

## `psiviz::DepthPicker`

`unproject(depth, x, y, width, height, dpr, view, proj)` → optional world position.

## `psiviz::PoseGizmo`

Holds `Eigen::Affine3f` pose. `beginFrame`, `hitTest`, `drag`, `isManipulating()`. Rendered by `PointCloudRenderer` after the point cloud.

## `psiviz::ViewerInputRouter`

Priority: gizmo drag > Shift+click pick > camera. Returns `InputAction` for the item to apply.

## `psiviz::PointCloudItem` (QML: `PointCloudScene`)

| Property | Description |
|----------|-------------|
| `blendWeight` | 0 = scalar colormap only, 1 = RGB only |
| `scalarMin`, `scalarMax` | JET range |
| `pointSize` | Screen-space point diameter |
| `gizmoEnabled` | Enable pose manipulation |
| `interactionEnabled` | Master switch (e.g. off during optimization) |

| Signal | Description |
|--------|-------------|
| `pointPicked(x, y, z)` | Shift+click depth pick |
| `poseChanged()` | Gizmo moved the pose matrix |

`poseMatrix()` / `setPoseMatrix()` expose the 4×4 pose as 16 doubles (column-major).
