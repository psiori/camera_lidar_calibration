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
