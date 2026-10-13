## Purpose

Fuses multiple masked structured lidar scans into a deskewed, merged dense cloud with user-controlled output resolution and live preview during processing. Each calibration dataset selects a **fusion method** via dropdown: **CT integrator** (`CtIntegrator`, default) or **CT-GICP SLAM** (`GlimSlam`), both implemented in `glim_cloud_fusion`. A single scan uses the same output resolution via downsampling only (no multi-scan fusion or deskew).

## ADDED Requirements

### Requirement: Masked input fusion

Fusion SHALL accept multiple structured scans after per-scan application of the global mask. Fusion SHALL be blocked if any scan fails geometry validation.

#### Scenario: Fuse masked scans (GLIM SLAM)

- **WHEN** the user runs fusion on valid masked scans with fusion backend `GlimSlam`
- **THEN** `glim_cloud_fusion::fuse_scans` uses `OdometryEstimationCT` (CT-GICP + fixed-lag iSAM2) and produces a merged point cloud

#### Scenario: Fuse masked scans (CT integrator)

- **WHEN** the user runs fusion on valid masked scans with fusion backend `CtIntegrator`
- **THEN** `glim_cloud_fusion::fuse_scans` runs the **CT integrator** path (per-scan CT-GICP deskew, sequential scan-to-model registration, voxel accumulation) **without** iSAM2 or multi-scan pose-graph optimization, and produces a merged point cloud

### Requirement: Fusion backend selection

Each calibration dataset SHALL store a fusion backend on `FusionParams` (`GlimSlam` or `CtIntegrator`). The default for new datasets and `apply_default_fusion_params` SHALL be `CtIntegrator`. When adding a dataset, `backend` SHALL copy from the first calibration dataset if one exists (so a session can mix methods per dataset). The Datasets tab SHALL expose a **Fusion method** dropdown for the **active** dataset before **Run fusion**. Changing method on one dataset SHALL NOT change another dataset’s stored backend. Changing backend SHALL not mutate paths or mask; the user MAY re-fuse with a different backend on the same dataset.

#### Scenario: Mixed methods in one session

- **WHEN** dataset A uses `CtIntegrator` and dataset B uses `GlimSlam`
- **THEN** each dataset’s saved `fusion_params.backend` and fuse runs are independent, and both produce viewer/export clouds with compatible intensity value range

#### Scenario: Switch backend and re-fuse

- **WHEN** the user changes fusion mode from `GlimSlam` to `CtIntegrator` and runs fusion again on the same scans
- **THEN** the new merged cloud reflects integrator behavior and SLAM smoother settings are not applied

### Requirement: CT integrator backend behavior

When `FusionParams::backend` is `CtIntegrator`, multi-scan fusion SHALL:

- Process scans in ascending PCD filename order (per `session`).
- For each scan after the first: estimate begin/end lidar poses with a **two-pose** continuous-time GICP factor against the growing target map, constant-velocity motion prediction from the previous scan, and Levenberg–Marquardt optimization (iteration counts per `design.md` integrator trace).
- Deskew points using the optimized poses, insert deskewed points into the target map, and accumulate points into an output voxel structure using pose interpolation along the sweep.
- SHALL NOT invoke `OdometryEstimationCT`, iSAM2, or any fixed-lag smoother.
- SHALL apply the same masked-input and scan-time normalization rules as the SLAM path where those steps are shared in `glim_cloud_fusion`.

Integrator-specific parameters (`target_num_points`, `min_distance`, `integrator_ivox_resolution`, `integrator_ivox_min_points_dist`, `integrator_ivox_lru_thresh`, etc.) SHALL be documented in `design.md` with vlcal-aligned defaults. `min_distance` SHALL mean minimum range when accumulating deskewed output points; `integrator_ivox_min_points_dist` SHALL mean minimum spacing inside the integrator scan-to-model iVox and SHALL NOT be aliased to `min_distance` or SLAM `ivox_min_points_dist`. Integrator fields SHALL be ignored when backend is `GlimSlam`. SLAM-only parameters (smoother lag, iSAM2 options, etc.) SHALL be ignored when backend is `CtIntegrator`.

Per-scan preprocessing for `CtIntegrator` SHALL use vlcal-equivalent random-grid subsampling (0.5 m grid) before the `target_num_points` cap.

#### Scenario: Rotation-relative timestamps with gaps

- **WHEN** PCD per-point times are in nanoseconds from sweep/rotation start and some angular sectors are missing from the saved cloud
- **THEN** fusion with default `time_unit = Nanoseconds` and `time_origin = ScanStart` SHALL deskew without subtracting the first point’s timestamp, provided `scan_duration` is set

#### Scenario: Integrator ignores smoother

- **WHEN** fusion runs with `CtIntegrator` and a non-default `smoother_lag` in session JSON
- **THEN** fusion outcome is unchanged from a run with default smoother fields

### Requirement: Headless fusion CLI backend

The `clc_fusion` executable SHALL accept a fusion backend flag (`--fusion-backend slam|integrator` or equivalent) and integrator-specific CLI overrides mirroring `FusionParams`, invoking the same `fuse_scans` API as `clc_app`.

#### Scenario: CLI integrator smoke test

- **WHEN** the user runs `clc_fusion` on a PCD directory with `--fusion-backend integrator` and `--output` set
- **THEN** a fused PCD is written without starting the Qt application

### Requirement: Single-scan passthrough

When exactly one scan is provided, fusion SHALL still produce an output cloud without running multi-scan CT-GICP SLAM. The path SHALL apply the global mask and downsample to `FusionParams.voxel_resolution` (same voxel grid as multi-scan output). It SHALL NOT perform motion deskew or merge across scans.

#### Scenario: One scan

- **WHEN** the user runs fusion with a single masked PCD
- **THEN** the system returns a downsampled cloud at the configured voxel resolution without error

#### Scenario: Single-scan UI on Datasets tab

- **WHEN** the active dataset has exactly one PCD in its catalog
- **THEN** scan-time and multi-scan fusion parameters are disabled in the UI; voxel resolution and **Run fusion** remain available for passthrough

### Requirement: User output voxel size

The user SHALL set output voxel resolution (e.g. 1 cm, 2 cm) via `FusionParams.voxel_resolution`. For multi-scan fusion this resolution is applied in post-SLAM `voxel_merge()`; for single-scan passthrough the same step downsamples the masked scan only. It is independent of internal GLIM iVox settings.

#### Scenario: 1 cm output

- **WHEN** the user sets voxel resolution to 0.01 m and runs fusion
- **THEN** the returned cloud is downsampled at approximately 1 cm voxels

### Requirement: Exposed fusion parameters

The **Datasets** tab SHALL expose every user-relevant field on `glim_cloud_fusion::FusionParams` for the **active** fusion backend, including fusion mode, scan-time deskew, preprocess neighbor count, backend-specific solver/map fields (see `design.md` trace tables for `GlimSlam` and `CtIntegrator`), and output `voxel_resolution`. Values SHALL be edited per calibration dataset and persisted in `session.json` under each dataset’s `fusion_params` only.

Each control SHALL have an in-app tooltip. For multi-scan datasets, the **Fusion method** dropdown SHALL be visible and SHALL reflect the active dataset’s `backend`. Parameter groups SHALL be **shown and wired** only for the selected method: CT-GICP / GLIM controls when `GlimSlam`; CT integrator controls when `CtIntegrator`; shared controls (scan time, `k_neighbors`, output voxel) per `design.md`. Inactive-backend controls MAY be hidden or disabled but SHALL NOT affect `fuse_scans` for the active method. Scan-time and backend-specific blocks SHALL be disabled when the active dataset is single-scan (passthrough); output voxel and **Run fusion** remain enabled per single-scan requirement.

### Requirement: Compatible fused output intensity

Fused clouds from either backend SHALL use the same intensity representation in memory, viewer colormaps, and exported PCD (field presence and nominal range compatible with the existing SLAM fusion path). Backend-specific processing SHALL NOT leave one dataset with a different intensity scale than another in the same session unless the user changes source scan content.

#### Scenario: Colormap after switching dataset

- **WHEN** the user views fused clouds from dataset A (integrator) and dataset B (SLAM) in the same session
- **THEN** intensity-driven coloring and blend behave without per-backend scale adjustment in the UI

#### Scenario: Adjust correspondence distance

- **WHEN** the user changes max correspondence distance before multi-scan fusion
- **THEN** the value is passed to `OdometryEstimationCT` on the next run

#### Scenario: iVox resolution distinct from output voxel

- **WHEN** the user sets `ivox_resolution` and `voxel_resolution` to different values
- **THEN** fusion uses iVox for scan-to-model matching during odometry and applies output downsampling only in the final `voxel_merge` step

#### Scenario: Fusion params binding audit

- **WHEN** a new `FusionParams` or `OdometryEstimationCTParams` field is added in glim
- **THEN** the change documents whether it is user-exposed, with trace from UI to `fuse_scans` (same discipline as NID params in `calibration-pipeline`)

### Requirement: Streaming fusion preview

Fusion SHALL invoke progress callbacks with partial accumulated geometry so the 3D viewer can show the cloud densifying during processing. Update frequency MAY be throttled (e.g. every few scans).

#### Scenario: Preview during fusion

- **WHEN** fusion is processing scan 3 of 10
- **THEN** the viewer displays points accumulated so far

### Requirement: Export fused PCD

The user SHALL export the fused cloud to PCD format.

#### Scenario: Export after fusion

- **WHEN** fusion completes and the user chooses export
- **THEN** a PCD file is written with the fused point cloud

### Requirement: Background execution

Fusion SHALL run on a worker thread; UI updates SHALL be marshalled to the main thread.

#### Scenario: UI responsive during fusion

- **WHEN** fusion is running
- **THEN** the main thread remains responsive and preview updates via queued signals

### Requirement: Per-dataset fusion

Fusion SHALL be invoked for one calibration dataset at a time using that dataset’s PCD directory, per-dataset scan-time and fusion parameters, and the session global mask. Input scans SHALL be ordered by ascending PCD filename (timestamp-in-filename; see `session`). Progress and output SHALL apply to the active dataset only.

#### Scenario: Fuse dataset two

- **WHEN** the user runs fusion with dataset B active on the Datasets tab
- **THEN** only dataset B’s scans are merged and dataset B becomes ready when complete

### Requirement: Release structured scans after fusion

After fusion completes successfully for a dataset, the application SHALL release in-memory copies of all source structured PCDs for that dataset. The fused cloud SHALL be persisted under the session directory (or configured path) and SHALL be loaded lazily and asynchronously for viewing. Structured scans SHALL be re-read from disk only when needed (for example mask preview or re-fuse).

#### Scenario: Memory after fuse

- **WHEN** fusion completes for a dataset with many scans
- **THEN** the viewer loads the fused cloud for display without keeping every input scan resident in RAM
