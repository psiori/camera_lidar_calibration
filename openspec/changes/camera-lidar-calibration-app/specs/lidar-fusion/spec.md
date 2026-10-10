## Purpose

Fuses multiple masked structured lidar scans into a deskewed dense cloud with user-controlled output resolution and live preview during processing.

## ADDED Requirements

### Requirement: Masked input fusion

Fusion SHALL accept multiple structured scans after per-scan application of the global mask. Fusion SHALL be blocked if any scan fails geometry validation.

#### Scenario: Fuse masked scans

- **WHEN** the user runs fusion on valid masked scans
- **THEN** `glim_cloud_fusion` performs CT-GICP SLAM and produces a merged point cloud

### Requirement: Single-scan passthrough

When exactly one scan is provided, fusion SHALL still produce an output cloud (deskew + voxel merge) without requiring multi-scan SLAM.

#### Scenario: One scan

- **WHEN** the user runs fusion with a single masked PCD
- **THEN** the system returns a voxel-merged cloud without error

### Requirement: User output voxel size

The user SHALL set output voxel resolution (e.g. 1 cm, 2 cm) via `FusionParams.voxel_resolution`. This resolution is applied in post-merge `voxel_merge()` and is independent of internal GLIM iVox settings.

#### Scenario: 1 cm output

- **WHEN** the user sets voxel resolution to 0.01 m and runs fusion
- **THEN** the returned cloud is downsampled at approximately 1 cm voxels

### Requirement: Exposed fusion parameters v1

The UI SHALL expose `FusionParams` fields: `voxel_resolution`, `num_threads`, `max_correspondence_distance`, and scan-time parameters (`time_unit`, `time_origin`, `scan_duration`). Internal GLIM odometry/submap parameters remain hardcoded. Each exposed field SHALL have an in-app tooltip (see `calibration-pipeline` tooltips requirement).

#### Scenario: Adjust correspondence distance

- **WHEN** the user changes max correspondence distance before fusion
- **THEN** the value is passed to the fusion pipeline on the next run

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
