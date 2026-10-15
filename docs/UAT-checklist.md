# Manual UAT checklist (camera-lidar-calibration-app)

Use a known session with structured PCDs, camera image, and intrinsics YAML.

## Session and dataset

- [ ] Load session directory; paths, scan time, mask path, and extrinsic appear in the UI
- [ ] Save session; reload and confirm round-trip (paths, mask, correspondences, `T_camera_lidar`)

## Scan time and fusion tooltips

- [ ] Hover time unit, time origin, scan duration; tooltips match `design.md`
- [ ] Hover fusion voxel/threads/max correspondence; tooltips are readable
- [ ] Run fusion on 2+ scans; viewer preview densifies during processing
- [ ] Run fusion on 1 scan; fused cloud succeeds (passthrough)

## Fusion backends and parameters (CT integrator / CT-GICP SLAM)

- [ ] **iVox vs output voxel (19.5):** On a multi-scan dataset, set integrator iVox resolution and output voxel to different values; fuse; change output voxel only and fuse again — point count shifts while fusion still succeeds. Save session, reload, confirm both values round-trip.
- [ ] **Intensity parity (20.6b):** Fuse the same masked multi-scan catalog with **CT integrator** and **CT-GICP SLAM**; exported PCD intensity min/max are in the same nominal range (no per-backend UI scale tweak). Viewer colormap looks consistent when switching datasets.
- [ ] **Backend isolation (20.8):** With **CT integrator** selected, toggle SLAM-only smoother / iSAM2 settings (or run `clc_fusion` with `--enable-smoother`); integrator output is unchanged. With **CT-GICP SLAM**, smoother settings affect odometry as expected.

CLI spot-check (optional, `sequence2_short` sample data): integrator voxel `0.01` vs `0.2` yielded ~225k vs ~24k points; integrator with `--enable-smoother` matched default point count; integrator and SLAM PCD intensities both spanned `1.0`–`255.0`.

## Mask and range image

- [ ] Edit mask in popout; keep/remove; save/load binary mask
- [ ] Fusion respects global mask on structured scans

## Initial guess and alignment

- [ ] Gizmo updates `T_camera_lidar_initial` and projected colors
- [ ] Shift+pick 3D + image click adds correspondences; estimate with 3+ pairs
- [ ] Run NID alignment; gizmo/picking disabled during run; live color update on progress

## Inspection

- [ ] Inspection tab uses separate paths stored in `inspection_datasets`
- [ ] Fuse inspection scans; 3D viewer shows cloud with saved `T_camera_lidar` colors
- [ ] Export colored PCD from inspection tab includes rgb

## Export

- [ ] Export fused PCD from calibration tab includes intensity and rgb

## 3D viewer interaction

- [ ] Viewer and mask colormap use dual-handle range sliders; scalar colors update while dragging (no geometry reload)
- [ ] With blend &gt; 0, colormap drag may lag slightly but final colors match on slider release
- [ ] Pointer over 3D view: `f` flies to point under cursor; `+` / `-` adjust point size; `h` frames cloud
- [ ] Left-drag up pitches view forward (PCL-style); overlay sliders still usable without stealing keys while dragging
