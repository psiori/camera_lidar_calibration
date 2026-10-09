## Why

The `camera_lidar_calibration` project has library foundations (WP1 `vlcal_align`, WP2 `glim_cloud_fusion`, WP3 `clc_core` shell) but lacks a complete interactive calibration workflow. Users need a Qt application that replaces the `direct_visual_lidar_calibration` viewer for multi-scan fusion, manual extrinsic estimation, NID optimization, and inspection — without Iridescence/VTK dependencies.

## What Changes

- Implement full calibration app UI on top of existing `clc_core` services
- Add **`psiviz`**, a self-contained Qt/QRhi 3D library in this repository (documented, calibration-agnostic; may be factored out later)
- Structured lidar range-image mask authoring (pre-calibration, global mask applied per scan)
- Point cloud viewer (`psiviz`) with PCL-style navigation, depth-buffer picking, and live color blending
- In-scene pose gizmo for `T_camera_lidar` (disabled during NID optimization)
- Manual initial guess (gizmo + Shift+click correspondences per dataset)
- NID calibration with live preview; compile-time extensible provider/optimizer traits
- LiDAR fusion UI with streaming preview, voxel output resolution, geometry validation
- Inspection datasets with overlay and colored PCD export
- Session persistence for datasets, mask, intrinsics, and calibrated extrinsic
- In-app tooltips on calibration/fusion controls (including scan-time deskew settings) so operators can learn parameters without external docs
- **App shell (agreed UX):** two tabs with segmented chrome; 50/50 calibration split; scrollable collapsible controls; viewer overlay; mask **popout** with I/O on main window; three-tab preprocessing workflow **not** pursued

## Capabilities

### New Capabilities

- `app-shell`: Two-tab main window, mask editor popout, session bar (see `design.md` UI decisions)
- `session`: Session model, calibration vs inspection datasets, single extrinsic, save/load
- `range-image-mask`: Structured range/intensity image, polygon mask, Y-stretch display, binary mask I/O
- `point-cloud-viewer`: QRhi renderer, colormap/blend, PCL navigation, depth picking, 20M-point target
- `pose-gizmo`: In-scene manipulation of `T_camera_lidar`; disabled during optimization
- `calibration-pipeline`: Manual initial guess, NID optimizer, compile-time provider traits, multi-dataset support
- `lidar-fusion`: Masked scan fusion, passthrough single scan, streaming preview, user voxel size
- `inspection`: Test datasets, camera-lidar overlay, export PCD with intensity and RGB

### Modified Capabilities

(none — no existing `openspec/specs/`)

## Impact

- **WP3** `psiviz` (in-repo): QRhi point viewer, trackball camera, depth picking, pose gizmo; README + API docs
- **WP3** `clc_app`: Two-tab QML shell, resizable mask editor window, `RangeImageItem`, integration with `psiviz` and `clc_core`
- **WP3** `clc_core`: Session schema, mask application, colormap/projection math, provider traits
- **WP2** `glim_cloud_fusion`: Fusion progress callbacks for streaming preview (may extend `FusionProgress`)
- **WP1** `vlcal_align`: Consumed via existing NID API; mask passed into alignment input
- **Dependencies**: Qt 6 Quick + QRhi (Metal on macOS); Conan-managed numeric stack unchanged
