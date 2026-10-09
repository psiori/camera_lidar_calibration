## Context

See `proposal.md`. The shell (`clc_app` + `clc_core`) exists with fusion/alignment workers and basic QML. Libraries are split across three work packages:

- **WP1** `vlcal_align`: headless NID calibration (`direct_visual_lidar_calibration`)
- **WP2** `glim_cloud_fusion`: CT-GICP SLAM + post-merge voxel downsampling
- **WP3** `camera_lidar_calibration`: Qt-free core + Qt/QML app + in-repo 3D viz library (`psiviz`)

Iridescence/VTK are not Qt-compatible; visualization logic from `PointsColorUpdater` and PCL `pcd_viewer` navigation is reimplemented in Qt.

## Goals / Non-Goals

**Goals:**

- End-to-end calibration: load data → mask → fuse → guess → optimize → inspect → export
- Single `T_camera_lidar` per session (camera pose in lidar frame)
- 20M-point viewer performance with depth-buffer picking
- PCL trackball navigation + pose gizmo (separate concerns)
- Global structured mask applied before fusion on every scan
- Self-contained **`psiviz`** library (documented, calibration-agnostic) inside this repository

**Non-Goals (v1):**

- ALIKED/LightGlue auto matching, target-based registration
- Sequence splitter from long recordings (deferred capability)
- Corner view-orientation gizmo (ImGuizmo `ViewManipulate`)
- Runtime plugin loading for guess/optimizer providers
- Exposing internal GLIM odometry/submap parameters beyond `FusionParams`
- Alt+wheel FOV zoom, rubber-band selection, stored viewpoint cycling
- Qt-free or API-agnostic graphics backend (may abstract later)
- Separate `psiviz` git repository (factor out only after v1 works)

## Decisions

### Architecture: `psiviz` + `clc_core` + `clc_app`

| Layer | Responsibility |
|-------|----------------|
| **`psiviz`** | Qt-coupled 3D library: QRhi point renderer, trackball camera, depth picking, pose gizmo, JET colormap shader, `QQuickFramebufferObject` items. No session, mask, or calibration types. |
| **`clc_core`** | Session, mask rasterization, `PointColorModel` (camera projection), provider traits, fusion/calibration orchestration |
| **`clc_app`** | QML workflow, `RangeImageItem`, app controller/workers, wires `clc_core` data into `psiviz` items |

**Repository layout (v1):** top-level CMake target `psiviz/` alongside `clc_core/` and `clc_app/`. Conan may list `psiviz` as a package consumed by `clc_app` even while sources live in-tree.

### 3D visualization library (`psiviz`)

- **Purpose:** Reusable Qt Quick 3D viewer building blocks for point clouds and pose manipulation, independent of camera–lidar calibration domain logic.
- **Qt coupling (v1):** Depends on Qt 6 Quick + QRhi. On macOS, rendering uses Qt’s RHI backend (Metal), not legacy OpenGL.
- **Scope (v1 only):**
  - `PointCloudView` / `PointCloudItem` — large point lists, scalar JET colormap, per-point RGB, blend uniform, point size
  - `TrackballCamera` — PCL `pcd_viewer` navigation semantics
  - `DepthPicker` — depth render target readback + unproject (`devicePixelRatio`-aware)
  - `PoseGizmo` — in-scene translate/rotate handles (ImGuizmo interaction pattern, QRhi drawing, no ImGui)
  - Input routing: gizmo hit → gizmo drag; else Shift+click → pick; else camera
- **Out of scope for `psiviz`:** mesh I/O, volume rendering, range-image UI, NID/fusion, session JSON, VTK/PCL viz.
- **Documentation:** `psiviz/README.md` (build, minimal example), `psiviz/docs/API.md` (public classes, threading, buffer upload contract), Doxygen on public headers.
- **Rejected:** ImGui/ImGuizmo in Qt Quick; Easy3D (GPL); VTK/PCL visualization; raw OpenGL-only stack on macOS.
- **Later:** Optional factor-out to its own repo and/or `RenderBackend` abstraction — not v1.

### Session model

```
Session
  T_camera_lidar, T_camera_lidar_initial
  global_mask (WxH binary, topology from reference scan)
  calibration_datasets[]: { image, cloud_path(s), correspondences[] }
  inspection_datasets[]: { image, cloud_path(s) }
```

One extrinsic for all datasets. Correspondences are per-dataset. `clc_app` binds gizmo pose ↔ `T_camera_lidar`.

### Range image and mask

- Native structured `width × height` grid; value = range or intensity per cell
- Not a camera projection; camera irrelevant at this stage
- Y-stretch is display-only for labeling squished vertical FOV
- Mask authored on one reference scan; saved as binary `width × height`; applied to every structured PCD before fusion
- Post-fusion cloud is unstructured; no further masking
- Implemented in **`clc_app`** / **`clc_core`**, not in `psiviz`

### Geometry validation

First loaded structured scan defines reference `(width, height)`. Any scan with mismatched dimensions → error, fusion disabled.

### Point cloud rendering (`psiviz` + QRhi)

- `QQuickFramebufferObject` renderer using QRhi (Metal on macOS)
- Static GPU buffers: positions, scalar channel; dynamic: projected RGB from host
- Fragment shader: `mix(colormap(scalar), rgb, blend_weight)`; bounds/uniform-only updates for slider
- Depth target for picking and fly-to; optional point-index target if depth collisions occur
- Target: 20M points (vlcal/Iridescence class performance)

### View camera (PCL trackball)

| Input | Action |
|-------|--------|
| Left drag | Rotate (trackball) |
| Middle / Shift+left drag | Pan |
| Right drag | Dolly |
| Wheel | Dolly |
| Cmd+drag (macOS) / Ctrl+drag | Spin (view-axis rotation) |
| `f` | Fly-to (depth pick → animate focal point) |
| `r` | Reset camera (fit scene) |
| `+` / `-` | Point size |

No Alt+wheel FOV zoom.

### Pose gizmo (`psiviz`)

- 3D widget in scene representing a generic `4×4` pose (app maps to `T_camera_lidar`)
- Does not move view camera
- Translate/rotate in local frame; QRhi axis/ring handles (no ImGui)
- **Disabled during NID optimization** (app sets `psiviz` interaction flags); re-enabled after completion
- Gizmo captures mouse when hit; otherwise navigation/picking apply

### Picking

- Shift+left click: depth-buffer pick → 3D point for correspondences
- Retina: multiply by `devicePixelRatio` before depth read
- No octree; depth buffer only (PCL/Iridescence pattern)

### Calibration providers (compile-time)

```cpp
struct InitialGuessTraits<ManualGuess> {
  static constexpr bool supports_multi_dataset = true;
};
struct OptimizerTraits<NidOptimizer> {
  static constexpr bool supports_multi_dataset = true;
};
```

v1: `ManualGuess` (gizmo + correspondences), `NidOptimizer` only.

### Fusion parameters (v1 UI)

Expose `FusionParams` only: `voxel_resolution`, `num_threads`, `max_correspondence_distance`, `scan_time`. Internal GLIM params remain hardcoded in `fusion.cpp`.

`voxel_resolution` is applied in `voxel_merge()` after SLAM (or on single-scan passthrough), independent of GLIM iVox.

### Fusion streaming preview

Extend `FusionProgress` (or parallel callback) with partial cloud snapshots; throttle uploads (every K scans or M points). Same `psiviz` renderer as final cloud.

### Color inspection (post-guess)

- Blend lidar scalar colormap with projected camera RGB (`PointsColorUpdater` logic in `clc_core`)
- Blend slider updates uniform only in `psiviz`
- Scalar channel: intensity, x, y, z; JET colormap with user min/max (3D widget separate from range-image bounds)
- Export PCD: xyz + intensity + rgb fields

## Risks / Trade-offs

| Risk | Mitigation |
|------|------------|
| 20M points GPU memory (~500MB+) | VBO pooling; optional LOD later |
| Depth pick collisions at 20M | Optional point-index render target |
| Qt coupling blocks non-Qt reuse | Accept for v1; extract interface when factoring out |
| Mask topology mismatch across files | Hard error on load; block fusion |
| Gizmo vs navigation conflict | `psiviz` input router: gizmo first; app disables gizmo during optimize |

## Open Questions

- **Sequence splitter**: Long recording → multiple datasets by timestamp matching. Deferred; needs separate change.
- **Gizmo during optimize**: Resolved — disabled while optimizer runs.
