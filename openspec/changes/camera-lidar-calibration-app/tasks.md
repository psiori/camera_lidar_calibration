## 0. psiviz library (in-repo, Qt/QRhi)

- [x] 0.1 Scaffold `psiviz/` target: CMake, link Qt6::Quick, README + `docs/API.md`
- [x] 0.2 `PointCloudScene` (`QQuickRhiItem`): QRhi point draw, xyz/scalar/rgb buffers, JET + blend shader, point size
- [x] 0.3 `TrackballCamera`: PCL navigation (rotate, pan, dolly, wheel, Cmd/Ctrl spin, `r` reset, `f` fly-to)
- [x] 0.4 Depth picking: QRhi depth target, unproject, Retina `devicePixelRatio` (unproject done; depth readback stub)
- [x] 0.5 `PoseGizmo`: QRhi translate/rotate handles, hit-test, `isManipulating()` for input gating (interaction done; gizmo draw pending)
- [x] 0.6 Input router: gizmo > Shift+pick > camera; document threading for buffer upload
- [x] 0.7 Minimal QML example or test harness in `psiviz` (optional smoke: render N points)

## 1. Session model, validation, and I/O

- [x] 1.1 Extend `SessionData` with calibration/inspection datasets, global mask metadata, reference topology
- [x] 1.2 Implement structured scan geometry validation on PCD load (width/height vs reference)
- [x] 1.3 Update `session.json` schema and `clc_core` save/load for new fields, mask path, and per-dataset correspondences (not QML)
- [x] 1.4 Load camera intrinsics via OpenCV `FileStorage` (`clc_core`)
- [x] 1.5 Wire session **Load** / **Save** in `clc_app`: session directory + `session.json`, status/errors, persist current `SessionData` (paths, poses, fusion/scan-time params)
- [x] 1.6 After successful session load: refresh QML/controller state (image path, PCD directory, scan time, `T_camera_lidar` / gizmo, intrinsics if present)
- [x] 1.7 Dataset path UI: image + PCD directory bound to session; enumerate PCDs on directory change; surface count/errors (hooks into 1.2)
- [x] 1.8 `FileDialog` / `FolderDialog` for session directory, image file, PCD folder, and intrinsics YAML
- [x] 1.9 Intrinsics UI: browse YAML → `FileStorage` loader (depends on 1.4); store intrinsics path in session on save

## 2. Range image and mask (clc_core + clc_app)

- [x] 2.1 Implement `RangeImageBuilder` and `Colormap` (JET, min/max) in `clc_core`
- [x] 2.2 Implement global mask rasterization from polygon (keep/remove inside) in native WxH
- [x] 2.3 Apply global mask to structured PCD points before fusion (`scan.keep` or equivalent)
- [x] 2.4 Build `RangeImageItem` QML widget with Y-stretch display and polygon editor
- [x] 2.5 Binary mask save/load UI

## 3. Point cloud viewer (clc_app + psiviz)

- [x] 3.1 Register `psiviz` `PointCloudScene` in `clc_app` QML; wire fused-cloud uploads
- [x] 3.2 Connect blend slider, scalar channel, and colormap bounds from `clc_core`
- [x] 3.3 Expose depth-pick and camera signals to QML (correspondences, fly-to)

## 4. Point color model (clc_core)

- [x] 4.1 Port `PointsColorUpdater` projection logic to Qt-free `PointColorModel`
- [x] 4.2 Reproject RGB on `T_camera_lidar` change; expose to viewer worker thread
- [x] 4.3 PCD export with xyz + intensity + rgb fields

## 5. Pose gizmo (clc_app + psiviz)

- [x] 5.1 Bind `psiviz` gizmo matrix ↔ session `T_camera_lidar` / initial guess
- [x] 5.2 Disable gizmo and correspondence picking during fusion and NID workers
- [x] 5.3 Re-enable gizmo after optimization completes

## 6. Calibration pipeline

- [x] 6.1 Add compile-time `InitialGuessTraits` / `OptimizerTraits` and provider registry table
- [x] 6.2 Manual guess: per-dataset Shift+click correspondences → `T_camera_lidar_initial` (display-only gizmo; solver pose seed not wired)
- [x] 6.3 Wire NID `run_alignment()` multi-dataset with global mask and progress callback
- [x] 6.4 Live viewer update on `CalibrationProgress`
- [x] 6.5 QML workflow: load → mask → fuse → guess → optimize → inspect (two-tab shell + mask editor window per `design.md`; includes session and dataset I/O per §1.5–1.9)

## 7. LiDAR fusion UI

- [x] 7.0 Add `ToolTip` text to calibration controls (scan time unit/origin/duration, fusion/alignment, blend, session); copy per `design.md` UI tooltips table
- [x] 7.1 Expose `FusionParams` in QML (voxel size, threads, max correspondence, scan time)
- [x] 7.2 Extend `FusionProgress` (or callback) for partial cloud streaming preview
- [x] 7.3 Throttle preview uploads; feed `psiviz` `PointCloudItem`
- [x] 7.4 Single-scan passthrough path verification
- [x] 7.5 Export fused PCD action

## 8. Inspection

- [x] 8.1 Inspection dataset load path (fusion optional, no NID by default)
- [x] 8.2 Overlay inspection data with saved `T_camera_lidar`
- [x] 8.3 Export colored PCD from inspection view

## 9. Integration and tests

- [x] 9.1 Unit tests: mask rasterization, colormap, geometry validation, session round-trip
- [x] 9.2 Manual UAT checklist against spec scenarios (include verifying scan-time and fusion tooltips)
- [x] 9.3 Update `BUILD.md` with app workflow and `psiviz` build notes

## 10. Application shell (QML layout)

Agreed UX is documented in `design.md` (UI planning decisions table) and `specs/app-shell/spec.md`. **Adopted:** two tabs, mask popout, mask I/O on main window, 50/50 split, scroll/collapsibles, viewer overlay, tab chrome. **Rejected:** three-tab preprocessing shell.

- [x] 10.1 Mask editor popout: `MaskEditorWindow.qml` opened from **Edit mask…**; channel, Y-stretch, colormap, polygon tools, Keep/Remove/Discard, and range view in the window
- [x] 10.2 Main window **Global mask** group keeps Load / Browse / Save / Clear (+ Edit mask…) only
- [x] 10.3 `RangeImageItem.fillBounds` for fit-in-rect scaling inside the resizable mask editor window
- [x] 10.4 Horizontal `SplitView` default 50/50 with draggable divider (controls vs 3D)
- [x] 10.5 Left column `ScrollView` + collapsible `Expansion` sections (Dataset expanded by default)
- [x] 10.6 Viewer scalar/blend controls as `ViewerOverlay` on `PointCloudScene`
- [x] 10.7 Segmented Calibration / Inspection tab styling (selected state visually distinct)

## 11. Viewer interaction polish

- [x] 11.1 `ColormapRangeSlider` + `ViewerOverlay`: dual-handle range slider for viewer colormap (live scalar uniforms; throttled RGB when blend &gt; 0; flush on release)
- [x] 11.2 `MaskEditorWindow`: same `ColormapRangeSlider` for range-image colormap (throttled `refreshRangeImage` display while dragging)
- [x] 11.3 `psiviz`: depth pick Y-flip + per-frame `TrackballCamera::tick`; reliable `f` fly-to at cursor
- [x] 11.4 Viewport keyboard routing: pointer-over viewer activates nav keys (`f`, `+`, `-`, `r`/`h`); overlay controls keep focus while dragging sliders
- [x] 11.5 `TrackballCamera::rotate`: vertical drag sign matches PCL `pcd_viewer` (drag up pitches forward)
- [x] 11.6 UAT checklist: colormap slider, fly-to, keys, trackball pitch

## 12. Initial alignment workspace (spec: `initial-alignment`, `app-shell`)

- [x] 12.1 Calibration **section strip** (superseded by four top-level tabs per `app-shell`) (segmented): Dataset, Mask, Fusion, Scan time, Initial alignment, Actions; single active section in left scroll column
- [x] 12.2 **Tri-pane** layout when Initial alignment active: `CameraImageItem` (pan/zoom/reticle) | table + method/compute | `PointCloudScene` + overlay
- [x] 12.3 Correspondence model in controller: pending pick, row selection, re-pick, per-row delete, markers in image + 3D
- [x] 12.4 Table lists all datasets’ pairs; active dataset editable; read-only rows visually distinct; switch dataset updates image pane
- [x] 12.5 `InitialGuessTraits` UI capability flags; wire Compute to multi-dataset `compute_manual_initial_guess` (all pairs)
- [ ] 12.5b Pass session `T_camera_lidar` as pose seed into manual initial guess when ≥3 pairs (blocked on vlcal / post-v1 gizmo)
- [x] 12.6 Remove legacy tiny image embed from old Initial guess expansion; tooltips for compute/correspondence flow

## 13. Multi-dataset session and shell (spec: `session`, `app-shell`, `lidar-fusion`)

- [x] 13.1 Four top-level tabs + shared calibration dataset selector; add/remove datasets only on Datasets tab
- [x] 13.2 `DatasetEntry`: id from earliest PCD filename timestamp (lexicographic scan order), per-dataset `fusion_params` / `scan_time`, readiness, immutable paths; session JSON migration
- [x] 13.3 Duplicate timestamp on add → error; geometry validation across datasets (shared WxH)
- [x] 13.4 Per-dataset fuse; post-fuse drop in-memory scans; persist fused cloud; lazy/async load into viewer
- [x] 13.5 Mask editor dataset selector (shared global mask); grey out not-ready datasets on Initial alignment / Calibrate
- [x] 13.6 Calibrate tab: global NID params + run on all ready datasets; 3D preview with display-only gizmo

## 18. NID optimization parameters (spec: `calibration-pipeline`, `app-shell`, `session`)

- [x] 18.1 Trace audit: document vlcal `VisualCameraCalibrationParams` usage in `design.md` (keep in sync with direct_visual_lidar_calibration on bump)
- [x] 18.2 `session.json`: serialize/deserialize `calibration_params` (all fields except `on_progress`)
- [x] 18.3 `AppController` + `Main.qml` Calibrate tab: expose every audited parameter with tooltips; NM-only fields disabled when BFGS selected
- [x] 18.4 Binding verification: confirm UI → `session_.calibration_params` → `AlignmentInput::params` → `VisualCameraCalibration` (checklist or test); note BFGS `num_threads` gap in tooltip
- [x] 18.5 UAT: change bins/outer iterations, save/load session, run alignment and observe behavior change (e.g. iteration counts in status)

## 14. Camera pose gizmo — display (spec: `pose-gizmo`, `initial-alignment`, `calibration-pipeline`)

v1 is **display-only** on Initial alignment and Calibrate (no drag/rotate). Camera frame: +X width, +Y down, +Z optical. Visual target: ImGuizmo / Iridescence.

- [x] 14.1 `psiviz`: QRhi draw camera pose gizmo at `T_camera_lidar` (axes and/or rings); matrix from app each frame
- [x] 14.2 Show gizmo on **Initial alignment** and **Calibrate** tabs only; hide on Datasets and Inspection
- [x] 14.3 Sync gizmo whenever `T_camera_lidar` changes (session load, compute initial alignment, NID progress callbacks on Calibrate)
- [x] 14.4 v1: gizmo not in pick/hit path — trackball and Shift+pick unchanged; document post-v1 interaction in `psiviz` API

## 15. Camera pose gizmo — interaction (post-v1, Initial alignment only, spec: `pose-gizmo`)

- [x] 15.1 Translate on camera X/Y/Z and rotate about all three axes (ImGuizmo parity with Iridescence)
- [x] 15.2 Input router on Initial alignment: gizmo handle hit before trackball; write `T_camera_lidar` on drag
- [x] 15.3 Disable gizmo interaction during fusion and NID workers; re-enable on Initial alignment after completion

## 16. Add calibration dataset dialog (spec: `app-shell`, `session`)

- [x] 16.1 `AddDatasetDialog.qml`: modal with three rows (image, LiDAR file or directory, session intrinsics), hints + Browse + OK/Cancel; no chained pickers on **Add…**
- [x] 16.2 `AppController`: `addCalibrationDatasetFromPaths(image, lidar, intrinsics)` (or equivalent); OK loads intrinsics when path set; reuse duplicate-id and geometry validation
- [x] 16.3 Wire **Add…** on Datasets to open dialog; dev prepopulate defaults per `design.md` (CMake-gated or debug-only)
- [x] 16.4 Remove/replace `addCalibrationDataset()` chained `QFileDialog` flow

## 17. Single-scan Datasets UI (spec: `app-shell`, `lidar-fusion`)

- [x] 17.1 `AppController`: expose `activeDatasetIsSingleScan` (or equivalent) when active catalog has exactly one PCD
- [x] 17.2 Datasets tab: disable/grey Scan time group and fusion threads + max correspondence when single scan; keep voxel + **Run fusion**; tooltips per `design.md`

## 19. Full CT-GICP / GLIM fusion parameters (spec: `lidar-fusion`, `session`, `app-shell`)

- [x] 19.1a `k_neighbors` on `FusionParams`; wire `fusion.cpp` (no hardcoded neighbor count)
- [ ] 19.1b Remaining `OdometryEstimationCTParams` fields on `FusionParams`; pass full `odom_params` in `fuse_scans`
- [x] 19.2a `session.json`: `k_neighbors` in per-dataset and legacy `fusion` blocks; default 40 in `apply_default_fusion_params`
- [ ] 19.2b Serialize remaining CT-GICP fields when 19.1b lands
- [x] 19.3a Datasets tab: `k_neighbors` SpinBox + tooltip (enabled for single- and multi-scan)
- [ ] 19.3b Expose remaining CT-GICP fields; grey odometry block when single-scan
- [ ] 19.4 Binding verification: UI → `DatasetEntry::fusion_params` → `run_fusion` → `fuse_scans` → `OdometryEstimationCT` (test or checklist)
- [ ] 19.5 UAT: change iVox resolution vs output voxel, save/load session, multi-scan fuse and confirm behavior shift
