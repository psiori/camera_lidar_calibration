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
- Runtime GLIM `config_odometry` JSON editing (app uses explicit `FusionParams` / per-dataset session fields instead)
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

**Pose seed:** `ManualGuessInput::gizmo_T_camera_lidar` is used only when fewer than three pairs (fallback). For ≥3 pairs, vlcal `PoseEstimation` does not yet take a seed; wiring a pose seed is post-v1 (`pose-gizmo` interaction). Display-only gizmo on Initial alignment / Calibrate visualizes `T_camera_lidar` in v1.

**Active dataset media:** Image preview, 3D cloud, and RGB projection must always match the active calibration dataset on Datasets / Initial alignment / Calibrate (`refreshActiveDatasetVisuals` in `AppController`).

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

### Fusion parameters (Datasets tab)

Expose **all user-relevant** fields that affect multi-scan CT-GICP fusion in `glim_cloud_fusion::fuse_scans`, stored per dataset in `DatasetEntry::fusion_params`, serialized in `session.json`, and passed unchanged into `fuse_scans`. Extend `glim_cloud_fusion::FusionParams` (in the `glim` repo) so nothing material is hardcoded in `fusion.cpp` except GLIM defaults when a field is unset.

**Output vs odometry:** `voxel_resolution` is **post-SLAM output** downsampling (`voxel_merge()`). It is independent of `ivox_resolution` (scan-to-model map used inside `OdometryEstimationCT`).

**glim trace audit** (`glim/glim_cloud_fusion/src/fusion.cpp` + `glim/include/glim/odometry/odometry_estimation_ct.hpp` — re-verify when bumping glim):

| UI group | Member (target on `FusionParams`) | Used where |
|----------|-------------------------------------|------------|
| Output | `voxel_resolution` | `voxel_merge()` after merge / single-scan passthrough |
| Scan time | `scan_time.*` | `normalize_lidar_scan_times` / deskew in `to_preprocessed` |
| Preprocess | `k_neighbors` | `PreprocessedFrame::k_neighbors` + neighbor search in `to_preprocessed` |
| CT-GICP odometry | `num_threads` | `OdometryEstimationCTParams::num_threads` |
| CT-GICP odometry | `max_correspondence_distance` | `OdometryEstimationCTParams::max_correspondence_distance` |
| iVox map | `ivox_resolution` | `OdometryEstimationCTParams::ivox_resolution` |
| iVox map | `ivox_min_points_dist` | `OdometryEstimationCTParams::ivox_min_points_dist` |
| iVox map | `ivox_lru_thresh` | `OdometryEstimationCTParams::ivox_lru_thresh` |
| Factors | `location_consistency_inf_scale` | CT-GICP graph weights |
| Factors | `constant_velocity_inf_scale` | CT-GICP graph weights |
| Solver | `lm_max_iterations` | Levenberg–Marquardt inner loop |
| Smoother | `smoother_lag` | Fixed-lag window [s] |
| Smoother | `use_isam2_dogleg` | iSAM2 optimizer choice |
| Smoother | `isam2_relinearize_skip` | iSAM2 relinearize cadence |
| Smoother | `isam2_relinearize_thresh` | iSAM2 relinearize threshold |

Not exposed in UI: `compute_covs` (marginal covariances for debugging; no effect on fused cloud export). Defaults for new fields SHALL match `OdometryEstimationCTParams()` constructed from GLIM `config_odometry` when not overridden in session.

**Implementation gate:** Before closing fusion-params tasks, trace each `FusionParams` field UI → `session` → `run_fusion` → `fuse_scans` → `OdometryEstimationCT(odom_params)` (and `to_preprocessed` for `k_neighbors`). Unit test: round-trip extended `fusion_params` in `session.json`.

**Datasets tab layout:** Keep **Fusion** group for output voxel + **Run fusion**. Add collapsible **CT-GICP / GLIM** subsection (multi-scan only): threads, max correspondence, iVox, factor weights, LM iterations, smoother fields, preprocess `k_neighbors`. Grey out entire CT-GICP block when `activeDatasetIsSingleScan` (same rule as today for threads/max corr).

### NID parameters (Calibrate tab)

Expose **all user-relevant** fields of `vlcal::VisualCameraCalibrationParams` on the **Calibrate** tab, stored in `SessionData::calibration_params`, serialized in `session.json`, and passed through `AlignmentInput` into `VisualCameraCalibration` without renaming or dropping fields.

**vlcal trace audit** (direct_visual_lidar_calibration `visual_camera_calibration.cpp` / `.hpp` — re-verify when bumping vlcal):

| UI field | Member | Used where |
|----------|--------|------------|
| Max outer iterations | `max_outer_iterations` | `calibrate()` outer `for` |
| Max inner iterations | `max_inner_iterations` | NM `max_iterations`; BFGS `max_num_iterations` |
| Threads | `num_threads` | NM cost `parallel for` only — **BFGS path does not pass this to OpenMP** (vlcal quirk; tooltip or fix upstream) |
| Δtrans / Δrot stop | `delta_trans_thresh`, `delta_rot_thresh` | Outer convergence after each inner solve |
| Disable Z-buffer culling | `disable_z_buffer_culling` | `ViewCulling` (`enable_depth_buffer_culling = !flag`) |
| NID bins | `nid_bins` | `CostCalculatorNID` / `NIDCost` |
| Registration type | `registration_type` | `NID_BFGS` vs `NID_NELDER_MEAD` |
| NM init step | `nelder_mead_init_step` | NM only |
| NM convergence | `nelder_mead_convergence_criteria` | NM only |

Not exposed as Calibrate controls: `on_progress` (app callback for live gizmo/RGB). Not in `VisualCameraCalibrationParams`: hardcoded trust region in `MultiNIDCost` (0.2 m / 2°) — out of scope unless vlcal adds fields later.

**vlcal preprocess CLI (not NID params):** `target_num_points` and standalone preprocess `min_distance` remain vlcal-only. In this app: fusion `voxel_resolution` 0.01 m, `num_threads` 15, `max_correspondence_distance` 0.5 m, `k_neighbors` 40 on `FusionParams` (UI + session JSON).

**App defaults (`apply_default_calibration_params`):** `nid_bins` 16, `registration_type` `nid_bfgs`, `num_threads` 15, `delta_trans_thresh` 0.005 m, `delta_rot_thresh` 0.01°, `disable_z_buffer_culling` false.

**Implementation gate:** Before closing the NID-params task, run a binding audit (grep / breakpoint / log once per field) confirming UI writes `session_.calibration_params` and `buildAlignmentInput` copies it to `input.params`. Add a short checklist in PR or `clc_core` test that default-constructed params round-trip through session JSON.

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

- Scrollable **NID / optimization** group: full `VisualCameraCalibrationParams` (see table above); disable or hide Nelder–Mead-only fields when BFGS is selected. Run alignment; 3D preview with display-only gizmo updating live during NID.

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
| Run fusion | Merge loaded PCD scans into one deskewed cloud (CT-GICP), using the global mask, scan-time settings, and CT-GICP / GLIM parameters. |
| Fusion voxel (output) | Final merged cloud downsampling resolution (m); not the same as iVox map resolution during odometry. |
| Fusion k-neighbors | Neighbors per point for covariance / CT-GICP preprocessing (vlcal preprocess default often 40; GLIM fusion today hardcodes 20 until §19). |
| iVox resolution | Voxel size of the target map used for scan-to-model CT-GICP matching during fusion. |
| iVox min point dist | Minimum spacing between points stored in an iVox cell. |
| iVox LRU thresh | iVox cache size before evicting old voxels. |
| Location / velocity consistency | Information weights on pose consistency factors in the CT-GICP graph. |
| LM max iterations | Max Levenberg–Marquardt iterations per scan insertion. |
| Smoother lag | Fixed-lag smoothing window length in seconds for multi-scan trajectory. |
| iSAM2 dogleg / relinearize | Advanced smoother optimizer settings (see GLIM `config_odometry`). |
| Run alignment | Optimize camera–lidar extrinsic (NID) using ready calibration datasets, global mask, and Calibrate NID settings. |
| Registration type | BFGS (default) or Nelder–Mead optimizer for each outer NID iteration. |
| NID bins | Histogram bins for mutual information cost (vlcal `nid_bins`). |
| Max outer / inner iterations | Outer extrinsic refinement loops and per-loop solver steps. |
| Convergence Δtrans / Δrot | Stop outer loop when pose change between iterations is below these thresholds. |
| Disable Z-buffer culling | Skip depth-buffer point culling before NID (more points, slower). |
| NID threads | OpenMP threads for Nelder–Mead multi-dataset cost; no effect on BFGS until vlcal is fixed. |
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
