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
  - `PoseGizmo` — in-scene camera-frame widget (QRhi); v1 display-only; post-v1 ImGuizmo-style interaction (no ImGui)
  - Input routing (post-v1): gizmo hit → gizmo drag; else Shift+click → pick; else camera. **v1:** gizmo not in hit path
- **Out of scope for `psiviz`:** mesh I/O, volume rendering, range-image UI, NID/fusion, session JSON, VTK/PCL viz.
- **Documentation:** `psiviz/README.md` (build, minimal example), `psiviz/docs/API.md` (public classes, threading, buffer upload contract), Doxygen on public headers.
- **Rejected:** ImGui/ImGuizmo in Qt Quick; Easy3D (GPL); VTK/PCL visualization; raw OpenGL-only stack on macOS.
- **Later:** Optional factor-out to its own repo and/or `RenderBackend` abstraction — not v1.

### Session model

```
Session
  T_camera_lidar, T_camera_lidar_initial
  global_mask (WxH binary, shared across calibration datasets, same sensor grid)
  reference_scan_width, reference_scan_height
  calibration_datasets[]: {
    id,  // timestamp from earliest PCD by filename (lexicographic = time); unique
    image_path, cloud_directory,  // immutable after add
    fusion_params, scan_time,
    fused_cloud_path, correspondences[], ready
  }
  inspection_datasets[]: { image, cloud_path(s) }
```

One extrinsic for all datasets. Correspondences are per-dataset. Gizmo display on **Initial alignment** and **Calibrate** (tracks `T_camera_lidar`; v1 non-interactive). Add/remove datasets only on **Datasets** tab; shared dataset selector elsewhere.

### Range image and mask

- Native structured `width × height` grid; value = range or intensity per cell
- Not a camera projection; camera irrelevant at this stage
- Y-stretch is display-only for labeling squished vertical FOV
- Mask shared session-wide; editor may preview range image from any calibration dataset; saved as binary `width × height`; applied to every structured PCD before fusion per dataset
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

- In-scene widget at `T_camera_lidar`; **camera frame**: +X image width, +Y image down, +Z optical axis
- **v1:** QRhi draw only (axes / ImGuizmo-style rings for readability); **no** pointer interaction; visible on Initial alignment + Calibrate; tracks `T_camera_lidar` on load, compute initial alignment, and live during/after NID on Calibrate
- **Post-v1:** ImGuizmo-equivalent translate + rotate on all camera axes (Iridescence parity); gizmo hit before trackball; disabled during NID
- Does not move view camera

### Picking

- Shift+left click: depth-buffer pick → 3D point for correspondences
- Retina: multiply by `devicePixelRatio` before depth read
- No octree; depth buffer only (PCL/Iridescence pattern)

### Initial alignment (`initial-alignment` spec)

**Layout:** Image | table/controls | 3D when the Initial alignment section is selected (see app-shell).

**Image pane (`CameraImageItem` in `clc_app`):** Reuse interaction patterns from `RangeImageItem` (uniform scale, letterbox, wheel zoom, drag pan). Reticle at cursor; click commits image side of a pair when a 3D pick is pending or when editing the selected table row.

**Correspondences:** Stored per calibration dataset in session; UI shows **all** rows, with non-active datasets read-only and visually distinct. Row select → highlight markers; re-pick updates that endpoint; delete removes markers and session entry. **Compute initial alignment** runs the active compile-time method over **all** datasets’ pairs (v1 minimum: three pairs total + intrinsics). Result writes `T_camera_lidar_initial` and working `T_camera_lidar`; blend slider on 3D inspects projection.

**Pose seed:** Manual solve uses session `T_camera_lidar` (`ManualGuessInput::gizmo_T_camera_lidar`). Gizmo on Initial alignment visualizes that pose in v1; interactive nudging is post-v1 (`pose-gizmo`).

**Extensibility:** `InitialGuessTraits` gains UI capability metadata (needs image, 3D, correspondence editor, pose seed) so a future feature-based provider can hide the table or populate it read-only. No runtime plugins.

### Calibration providers (compile-time)

```cpp
struct InitialGuessTraits<ManualGuess> {
  static constexpr bool supports_multi_dataset = true;
  // UI: image + 3D + correspondence editor + pose seed
};
struct OptimizerTraits<NidOptimizer> {
  static constexpr bool supports_multi_dataset = true;
};
```

v1: `ManualGuess` (pose seed + correspondences), `NidOptimizer` only.

### Fusion parameters (v1 UI)

Expose `FusionParams` only: `voxel_resolution`, `num_threads`, `max_correspondence_distance`, `scan_time`. Internal GLIM params remain hardcoded in `fusion.cpp`.

`voxel_resolution` is applied in `voxel_merge()` after multi-scan SLAM. On single-scan passthrough, `voxel_merge()` downsamples the masked scan only (no SLAM or deskew). Independent of GLIM iVox.

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
| Top-level navigation | **Four tabs:** **Datasets**, **Initial alignment**, **Calibrate**, **Inspection** | **Adopted** |
| Two-tab Calibration + Inspection only | Older shell | **Superseded** |
| Shared calibration dataset selector | Same tabs on Datasets / Initial alignment / Calibrate; add/remove only on Datasets | **Adopted** |
| Dataset id | Timestamp from earliest PCD by **filename** (lexicographic = chronological); duplicate → error | **Adopted** |
| PCD scan order | Ascending filename within each dataset directory | **Adopted** |
| Paths after add | Immutable; delete dataset to change | **Adopted** |
| Per-dataset fusion + scan_time | Copy from first dataset on add; edited on Datasets tab | **Adopted** |
| Mask | Session-global; editor has dataset selector for preview only | **Adopted** |
| Post-fusion memory | Fused cloud to disk; lazy/async viewer load; drop resident multi-scan buffers | **Adopted** |
| Gizmo display | Initial alignment + Calibrate (v1 display-only); updates with `T_camera_lidar` | **Adopted** |
| Gizmo interaction | Post-v1 on Initial alignment only; Calibrate stays display-only | **Adopted** |
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
| **Initial alignment** layout | Tri-pane: image \| table \| 3D on its own top-level tab | **Adopted** |
| **Datasets** tab | Fuse, mask I/O, intrinsics, per-dataset params; 50/50 with 3D preview optional | **Adopted** |
| Correspondence table | All datasets’ pairs in one table; only active dataset rows editable; compute uses all pairs | **Adopted** |
| Initial-alignment methods | Compile-time registry + per-method UI capability flags; v1 manual only | **Adopted** |
| **Add calibration dataset** | Modal `AddDatasetDialog.qml` (image + LiDAR file/dir + session intrinsics); not chained `FileDialog`s | **Adopted** |
| **Single-scan Datasets UI** | Grey out Scan time + multi-scan Fusion fields when catalog has one PCD; keep voxel + **Run fusion** | **Adopted** |

#### Target shell (multi-dataset)

**Datasets tab**

- Shared **dataset tabs** + **Add…** / Remove. **Add…** opens **`AddDatasetDialog.qml`**: three hinted rows (camera image, LiDAR scans as directory or single `.pcd`, session intrinsics YAML), browse per row, OK/Cancel. Active dataset: show baked paths (read-only), fuse, per-dataset scan time + fusion params, global mask I/O, Edit mask…, intrinsics (session). **`SplitView`** ~50/50 with `PointCloudScene` showing active dataset’s ready cloud.

**Add calibration dataset dialog (v1)**

- `Dialog` or small `Window` centered on the main window; width ~520px.
- Rows (label + hint + path + Browse):
  - **Camera image** — “Still photo taken with the lidar camera for this capture.”
  - **LiDAR scans** — “Folder of structured `.pcd` sweeps, or one `.pcd` for a single scan.”
  - **Camera intrinsics (session)** — “OpenCV YAML (`FileStorage`); shared by all calibration datasets.”
- OK calls `AppController` add API with the three paths; Cancel clears dialog-local state only.
- **Development defaults:** On first open (debug / local dev builds), pre-fill path fields with the team’s sample data so **Add** can be exercised without re-browsing:
  - Image: `/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/sequence1/2026-10-01T18_42_13_624Z.png`
  - LiDAR: `/Users/rasmus/Downloads/pbb-muc-255-1-fasssauna-high-freq-approach/sequence1.pcd`
  - Intrinsics: `/Users/rasmus/Documents/Projects/autocrane-core/config/pbb/muc/shared/calibration/cam_bridge_narrow_rotated_upright.yml`
  - Production builds SHOULD ship with empty defaults (or gate prepopulate behind `#ifdef` / `CLC_DEV_DEFAULT_PATHS` CMake option).

**Initial alignment tab**

- Same dataset selector (not-ready entries disabled + tooltip). Tri-pane: image | table + compute | 3D + gizmo (when implemented) + overlay.

**Calibrate tab**

- Global NID params + Run alignment; 3D preview with display-only gizmo updating live during NID.

**Inspection tab**

- Unchanged scope; inspection datasets only.

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
| Add… (Datasets) | Open the add-dataset dialog: pick camera image, LiDAR folder or single PCD, and session intrinsics before creating the dataset. |
| Add dialog — Camera image | One image file for this dataset (projection, range-image preview, NID). |
| Add dialog — LiDAR scans | Directory of `.pcd` files (multi-scan) or one `.pcd` file (single scan). |
| Add dialog — Intrinsics | Session-wide camera YAML; loaded on OK if changed. |
| Scan time (single PCD) | Disabled when the active dataset has one scan — deskew settings apply only to multi-scan fusion. |
| Fusion threads / max corr. (single PCD) | Disabled when the active dataset has one scan — passthrough uses voxel downsampling only. |

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
