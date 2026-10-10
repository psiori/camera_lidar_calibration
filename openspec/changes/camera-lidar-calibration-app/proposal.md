## Why

The `camera_lidar_calibration` project has library foundations (WP1 `vlcal_align`, WP2 `glim_cloud_fusion`, WP3 `clc_core` shell) but lacks a complete interactive calibration workflow. Users need a Qt application that replaces the `direct_visual_lidar_calibration` viewer for multi-scan fusion, manual extrinsic estimation, NID optimization, and inspection — without Iridescence/VTK dependencies.

## What Changes

- Implement full calibration app UI on top of existing `clc_core` services
- Add **`psiviz`**, a self-contained Qt/QRhi 3D library in this repository (documented, calibration-agnostic; may be factored out later)
- Structured lidar range-image mask authoring (pre-calibration, global mask applied per scan)
- Point cloud viewer (`psiviz`) with PCL-style navigation, depth-buffer picking, and live color blending
- Camera pose gizmo on Initial alignment and Calibrate (v1: display at `T_camera_lidar`; post-v1: ImGuizmo-style interaction on Initial alignment only)
- Initial alignment workspace: tri-pane (image | correspondence table | 3D), segmented calibration sections, manual correspondences with unified method interface (gizmo-seeded solve; feature methods later)
- NID calibration with live preview; compile-time extensible provider/optimizer traits
- LiDAR fusion UI with streaming preview, voxel output resolution, geometry validation
- Inspection datasets with overlay and colored PCD export
- Session persistence for datasets, mask, intrinsics, and calibrated extrinsic
- In-app tooltips on calibration/fusion controls (including scan-time deskew settings) so operators can learn parameters without external docs
- **App shell:** four tabs (Datasets · Initial alignment · Calibrate · Inspection); shared calibration-dataset selector; multi-dataset fuse/mask/params on Datasets; tri-pane initial alignment; automatic NID on Calibrate; mask popout with dataset preview selector
- **Multi-dataset session:** timestamp id, immutable paths, per-dataset fusion/scan_time, shared mask and sensor grid, lazy fused-cloud loading

## Capabilities

### New Capabilities

- `app-shell`: Four-tab workflow shell, shared dataset selector, mask editor popout (see `design.md` UI decisions)
- `session`: Session model, calibration vs inspection datasets, single extrinsic, save/load
- `range-image-mask`: Structured range/intensity image, polygon mask, Y-stretch display, binary mask I/O
- `point-cloud-viewer`: QRhi renderer, colormap/blend, PCL navigation, depth picking, 20M-point target
- `pose-gizmo`: Camera-frame pose indicator on Initial alignment and Calibrate (v1 display-only; post-v1 interaction on Initial alignment)
- `initial-alignment`: Tri-pane manual picking UI, correspondence table across datasets, compile-time initial-alignment methods, compute shared extrinsic from all pairs
- `calibration-pipeline`: NID optimizer, compile-time provider traits, orchestration with `initial-alignment` for manual guess
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
