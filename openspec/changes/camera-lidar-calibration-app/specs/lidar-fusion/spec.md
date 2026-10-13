## Purpose

Fuses multiple masked structured lidar scans into a deskewed, merged dense cloud with user-controlled output resolution and live preview during processing. A single scan uses the same output resolution via downsampling only (no multi-scan SLAM or deskew).

## ADDED Requirements

### Requirement: Masked input fusion

Fusion SHALL accept multiple structured scans after per-scan application of the global mask. Fusion SHALL be blocked if any scan fails geometry validation.

#### Scenario: Fuse masked scans

- **WHEN** the user runs fusion on valid masked scans
- **THEN** `glim_cloud_fusion` performs CT-GICP SLAM and produces a merged point cloud

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

The **Datasets** tab SHALL expose every user-relevant field on `glim_cloud_fusion::FusionParams` that affects multi-scan CT-GICP fusion, including scan-time deskew, preprocess neighbor count, all `glim::OdometryEstimationCTParams` fields wired through `fuse_scans` (see `design.md` trace table), and output `voxel_resolution`. Values SHALL be edited per calibration dataset and persisted in `session.json` under each dataset’s `fusion_params` (and session-level legacy `fusion` block for migration).

Each control SHALL have an in-app tooltip. CT-GICP / GLIM controls SHALL be disabled when the active dataset is single-scan (passthrough); output voxel and **Run fusion** remain enabled per single-scan requirement.

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
