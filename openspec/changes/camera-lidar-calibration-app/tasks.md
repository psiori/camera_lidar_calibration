## 0. psiviz library (in-repo, Qt/QRhi)

- [ ] 0.1 Scaffold `psiviz/` target: CMake, link Qt6::Quick, README + `docs/API.md`
- [ ] 0.2 `PointCloudItem` (`QQuickFramebufferObject`): QRhi point draw, xyz/scalar/rgb buffers, JET + blend shader, point size
- [ ] 0.3 `TrackballCamera`: PCL navigation (rotate, pan, dolly, wheel, Cmd/Ctrl spin, `r` reset, `f` fly-to)
- [ ] 0.4 Depth picking: QRhi depth target, unproject, Retina `devicePixelRatio`
- [ ] 0.5 `PoseGizmo`: QRhi translate/rotate handles, hit-test, `isManipulating()` for input gating
- [ ] 0.6 Input router: gizmo > Shift+pick > camera; document threading for buffer upload
- [ ] 0.7 Minimal QML example or test harness in `psiviz` (optional smoke: render N points)

## 1. Session and validation

- [ ] 1.1 Extend `SessionData` with calibration/inspection datasets, global mask metadata, reference topology
- [ ] 1.2 Implement structured scan geometry validation on PCD load (width/height vs reference)
- [ ] 1.3 Update session JSON save/load for new fields and per-dataset correspondences
- [ ] 1.4 Load camera intrinsics via OpenCV `FileStorage`

## 2. Range image and mask (clc_core + clc_app)

- [ ] 2.1 Implement `RangeImageBuilder` and `Colormap` (JET, min/max) in `clc_core`
- [ ] 2.2 Implement global mask rasterization from polygon (keep/remove inside) in native WxH
- [ ] 2.3 Apply global mask to structured PCD points before fusion (`scan.keep` or equivalent)
- [ ] 2.4 Build `RangeImageItem` QML widget with Y-stretch display and polygon editor
- [ ] 2.5 Binary mask save/load UI

## 3. Point cloud viewer (clc_app + psiviz)

- [ ] 3.1 Register `psiviz` `PointCloudItem` in `clc_app` QML; wire worker-thread uploads
- [ ] 3.2 Connect blend slider, scalar channel, and colormap bounds from `clc_core`
- [ ] 3.3 Expose depth-pick and camera signals to QML (correspondences, fly-to)

## 4. Point color model (clc_core)

- [ ] 4.1 Port `PointsColorUpdater` projection logic to Qt-free `PointColorModel`
- [ ] 4.2 Reproject RGB on `T_camera_lidar` change; expose to viewer worker thread
- [ ] 4.3 PCD export with xyz + intensity + rgb fields

## 5. Pose gizmo (clc_app + psiviz)

- [ ] 5.1 Bind `psiviz` gizmo matrix ↔ session `T_camera_lidar` / initial guess
- [ ] 5.2 Disable gizmo and correspondence picking during fusion and NID workers
- [ ] 5.3 Re-enable gizmo after optimization completes

## 6. Calibration pipeline

- [ ] 6.1 Add compile-time `InitialGuessTraits` / `OptimizerTraits` and provider registry table
- [ ] 6.2 Manual guess: gizmo + per-dataset Shift+click correspondences → `T_camera_lidar_initial`
- [ ] 6.3 Wire NID `run_alignment()` multi-dataset with global mask and progress callback
- [ ] 6.4 Live viewer update on `CalibrationProgress`
- [ ] 6.5 QML workflow: load → mask → fuse → guess → optimize → inspect

## 7. LiDAR fusion UI

- [ ] 7.1 Expose `FusionParams` in QML (voxel size, threads, max correspondence, scan time)
- [ ] 7.2 Extend `FusionProgress` (or callback) for partial cloud streaming preview
- [ ] 7.3 Throttle preview uploads; feed `psiviz` `PointCloudItem`
- [ ] 7.4 Single-scan passthrough path verification
- [ ] 7.5 Export fused PCD action

## 8. Inspection

- [ ] 8.1 Inspection dataset load path (fusion optional, no NID by default)
- [ ] 8.2 Overlay inspection data with saved `T_camera_lidar`
- [ ] 8.3 Export colored PCD from inspection view

## 9. Integration and tests

- [ ] 9.1 Unit tests: mask rasterization, colormap, geometry validation, session round-trip
- [ ] 9.2 Manual UAT checklist against spec scenarios
- [ ] 9.3 Update `BUILD.md` with app workflow and `psiviz` build notes
