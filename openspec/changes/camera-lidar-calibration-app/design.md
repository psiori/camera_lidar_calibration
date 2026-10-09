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

### Session and dataset I/O (v1 UI)

`clc_app` exposes paths and session persistence in `Main.qml`:

- **Session directory** + **Load** / **Save**: read/write `session.json` under the chosen folder (intrinsics, extrinsics, fusion/scan-time params, dataset paths, mask path when modeled).
- **Image path** and **PCD directory**: per-dataset inputs; PCD directory is scanned for `.pcd` files when set or after session load.
- **Load intrinsics** (planned): OpenCV YAML via `FileStorage`; path stored in session on save.

After **Load**, the controller SHALL push restored values into bound QML fields (paths, scan time, gizmo/extrinsic) so the UI matches `SessionData`, not only in-memory state behind text fields the user typed earlier.

### Main window shell (`clc_app` QML)

#### UI planning decisions (conversation record)

| Topic | Decision | Status |
|-------|----------|--------|
| Top-level navigation | **Two tabs only:** **Calibration** and **Inspection** (`TabBar` + `StackLayout`) | **Adopted** |
| Three-tab shell (Loading/preprocessing · Calibration · Inspection) | Dedicated preprocessing tab with full-screen 2D mask and no 3D viewer | **Rejected** — user chose to keep two tabs |
| Mask authoring layout | **Resizable popout window** from **Edit mask…**; user adjusts window size and Y-stretch for aspect | **Adopted** (`MaskEditorWindow.qml`) |
| Mask file I/O on main window | **Load**, **Browse**, **Save**, **Clear** stay in **Global mask** on Calibration tab | **Adopted** |
| Mask editing controls in popout | Channel, Y-stretch, colormap min/max, polygon draw tools, **Keep** / **Remove** / **Discard** after close polygon | **Adopted** |
| Range image in main column | No inline `RangeImageItem` on main window (frees horizontal space; avoids lidar aspect blowing layout) | **Adopted** |
| Inspection tab scope | Paths + fuse only; **no** new inspection features tied to this UI pass | **Adopted** |
| Default **50 / 50** `SplitView` (controls vs 3D) | Equal split with draggable divider | **Adopted** |
| Collapsible control sections (Dataset open by default) | Scroll + `Expansion` sections | **Adopted** |
| Viewer scalar/blend as **overlay** on `PointCloudScene` | `ViewerOverlay` on 3D, not left column | **Adopted** |
| Strong tab chrome (segmented selected state) | Calibration vs Inspection visually obvious | **Adopted** |
| Session bar | Directory + **Load** / **Save** below tab content, visible on both tabs | **Adopted** |

#### Implemented shell (v1)

**Calibration tab**

- Horizontal **`SplitView`** default **50 / 50**: left **`ScrollView`** with collapsible **`Expansion`** sections (Dataset expanded by default); right column for `PointCloudScene` with **`ViewerOverlay`** (scalar, colormap, blend).
- No range image or polygon editor in this column.

**Global mask editor window**

- **Edit mask…** opens a separate **resizable** top-level window (`MaskEditorWindow.qml`).
- Popout: range/intensity channel, Y-stretch, colormap bounds, polygon tools (close / undo / cancel draw), then **Keep** or **Remove** for a closed polygon (or **Discard polygon**), and `RangeImageItem` with **`fillBounds`** (letterboxed, uniform scale).
- **Load mask**, **Browse mask…**, **Save mask**, **Clear mask**, and **Edit mask…** on the main **Global mask** group only.

**Inspection tab**

- Dataset paths and fuse action; same shared session and 3D viewer wiring as today. No mask popout requirement on this tab.

**Responsive behavior**

- Left column uses `ScrollView` so controls are not clipped when the window is short.
- `RangeImageItem` in the mask editor MUST use fit-in-rect scaling so wide structured scans do not dictate main-window geometry.
- Top-level **Calibration / Inspection** switch uses segmented buttons with clear selected styling (not plain unstyled `TabBar` labels).

### UI tooltips (v1)

Use Qt Quick `ToolTip` (or `ToolTip.visible` on hover) on labeled controls in `clc_app` QML. Keep strings in QML or a single `Tooltips.qml` / `qsTr` resource for easy edit; design lists canonical v1 copy below.

**Scan time** (maps to `glim_cloud_fusion::ScanTimeParams` for deskew / motion compensation):

| Control | Tooltip (v1 copy) |
|---------|-------------------|
| Time unit | Unit of the per-point `time` values in your PCD files. Choose Seconds or Nanoseconds to match how timestamps are stored. Used when fusing scans to correct motion during each sweep. |
| Time origin | How each scan’s timestamps are zeroed before deskew. **First point:** time 0 at the earliest point in the scan. **Scan start:** time 0 at the scan header stamp; you must also set scan duration. |
| Scan duration (s) | Length of one full lidar sweep in seconds. Required when time origin is Scan start. Ignored or optional when using First point, depending on PCD metadata. |

**Other controls (minimum v1):**

| Control | Tooltip (v1 copy) |
|---------|-------------------|
| Run fusion | Merge loaded PCD scans into one deskewed cloud (CT-GICP), using the global mask and scan-time settings. |
| Run alignment | Optimize camera–lidar extrinsic (NID) using the current image, fused cloud, and correspondences. |
| Blend | Mix lidar scalar color with projected camera RGB (0 = lidar only, 1 = camera only). |
| Load / Save session | Load or save datasets, mask, intrinsics, and extrinsic from a session directory. |
| Session directory | Folder containing `session.json` and related calibration data. |
| Image path | Path to the camera image for this dataset (projection and NID alignment). |
| PCD directory | Folder of structured `.pcd` scans to fuse (calibration or inspection). |
| Load intrinsics | OpenCV camera intrinsics YAML (`FileStorage`); used for projection and NID. |

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
