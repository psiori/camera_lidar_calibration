## Purpose

Defines persistent session state for camera-LiDAR calibration: one rigid extrinsic, calibration and inspection datasets, and global mask metadata.

## ADDED Requirements

### Requirement: Single extrinsic per session

The session SHALL store exactly one `T_camera_lidar` (camera pose in lidar frame) shared by all calibration and inspection datasets in that session. It SHALL also store session-global `vlcal::VisualCameraCalibrationParams` as `calibration_params` for the Calibrate tab and NID runs.

#### Scenario: Multiple datasets same extrinsic

- **WHEN** the user adds a second calibration dataset
- **THEN** the system applies the same `T_camera_lidar` to both datasets without per-dataset extrinsic overrides

#### Scenario: NID params in session file

- **WHEN** the user saves a session after editing NID settings on Calibrate
- **THEN** `session.json` includes serialized `calibration_params` and reload restores them to the Calibrate UI

### Requirement: Calibration and inspection datasets

The session SHALL distinguish calibration datasets (used for initial guess and NID optimization) from inspection datasets (overlay and export only, no optimization unless explicitly promoted).

#### Scenario: Inspection dataset overlay

- **WHEN** the user loads an inspection dataset after calibration
- **THEN** the system applies the saved `T_camera_lidar` for visualization without running NID

### Requirement: Per-dataset image and cloud pairing

Each calibration dataset SHALL contain exactly one image path, one PCD directory (baked in at creation), and one point cloud used for labeling and optimization: either the fused output or a single-scan passthrough cloud. Paths SHALL NOT be changed after the dataset is added; the user SHALL remove the dataset and add a new one to use different paths.

#### Scenario: Fused cloud pairing

- **WHEN** fusion completes for a set of masked scans
- **THEN** the resulting unstructured cloud is paired with that dataset’s image as the dataset’s working cloud (and persisted path when saved)

#### Scenario: Single-scan ready

- **WHEN** a dataset has exactly one structured PCD and fusion passthrough has run (or equivalent single-scan output exists)
- **THEN** the dataset is **ready** without multi-scan SLAM

### Requirement: PCD scan order by filename timestamp

Within a dataset’s PCD directory, scans SHALL be ordered by **filename** ascending. Filenames SHALL encode capture time in a sortable string format so lexicographic order matches chronological scan order. Fusion and “first scan” rules SHALL use this ordering, not directory enumeration order or file modification time.

#### Scenario: Multi-scan fuse order

- **WHEN** fusion runs on a directory with several `.pcd` files
- **THEN** scans are processed in ascending filename order

### Requirement: Dataset identity from first scan timestamp

Each calibration dataset SHALL be identified by the timestamp parsed from the **first** PCD in filename order (the earliest scan in that directory). That value SHALL be unique among all calibration datasets in the session. If a new dataset would duplicate an existing id, the system SHALL show an error and SHALL NOT add the dataset.

#### Scenario: Duplicate timestamp rejected

- **WHEN** the user adds a dataset whose first-scan filename timestamp matches another calibration dataset’s id
- **THEN** the add fails with an error and the session’s dataset list is unchanged

#### Scenario: Add dialog validates before create

- **WHEN** the user confirms **Add calibration dataset** with image and LiDAR paths filled in
- **THEN** the system scans the LiDAR path (file or directory), assigns `id` from the first PCD in filename order, rejects duplicates and geometry mismatches with an error message in the dialog or status bar, and only then appends the dataset

### Requirement: Per-dataset fusion and scan-time parameters

Each calibration dataset SHALL store its own `FusionParams` and `ScanTimeParams`. `FusionParams` SHALL include the full CT-GICP / GLIM surface documented in `lidar-fusion` and `design.md` (extended struct in `glim_cloud_fusion`, not only voxel/threads/max correspondence). When the user adds a new calibration dataset, those fields SHALL be initialized from the first calibration dataset (or session defaults if the list was empty). Session-level legacy fusion/scan-time fields MAY remain for migration but the UI SHALL edit per-dataset values on the Datasets tab.

#### Scenario: Extended fusion_params in session file

- **WHEN** the user saves a session after editing CT-GICP settings on Datasets
- **THEN** `session.json` includes the extended `fusion_params` object for each dataset and reload restores them to the UI

#### Scenario: Copy on add

- **WHEN** the user adds a second calibration dataset
- **THEN** its fusion and scan-time controls start equal to dataset one’s values until the user changes them

### Requirement: Dataset readiness

A calibration dataset SHALL be **ready** for initial alignment and NID when it has a valid image path, baked PCD directory, matching session sensor geometry, and a usable cloud: either fusion has completed (fused cloud in memory or on disk) or exactly one scan was processed through the single-scan passthrough path. Not-ready datasets SHALL remain in the session but SHALL be excluded from picking and from optimization until ready.

#### Scenario: Multi-scan not fused

- **WHEN** a dataset has two or more PCDs and no fused output yet
- **THEN** it is not ready and Initial alignment treats it as disabled in the shared selector

### Requirement: Shared sensor geometry

All structured PCDs across all calibration datasets in a session SHALL share the same `width` and `height` (same physical lidar). The session SHALL store reference dimensions; any scan or dataset that violates them SHALL error and block fusion for that dataset.

#### Scenario: Mismatched scan dimensions

- **WHEN** a user adds a dataset whose scans do not match the session reference width and height
- **THEN** the system shows an error and does not treat that dataset as valid until resolved

### Requirement: Session persistence

The session SHALL be saveable and loadable to a directory including intrinsics, extrinsics, global mask path, per-dataset fusion and scan-time settings, dataset ids and paths, fused cloud paths, and per-dataset correspondences.

#### Scenario: Save and reload

- **WHEN** the user saves the session and later loads it
- **THEN** all datasets, mask reference, and `T_camera_lidar` are restored

### Requirement: Session load and save from UI

The UI SHALL expose session directory, Load, and Save on the main window.

#### Scenario: Load session from UI

- **WHEN** the user chooses a session directory and activates Load
- **THEN** `session.json` is read and the UI shows restored dataset paths, fusion/scan-time settings, intrinsics (if saved), and `T_camera_lidar` (including gizmo pose)

#### Scenario: Save session from UI

- **WHEN** the user activates Save for the current session directory
- **THEN** `session.json` reflects the current session state (paths, parameters, extrinsic, and mask reference when applicable)

### Requirement: Structured scan geometry validation

All structured input scans in a session MUST share the same `width` and `height` as the session reference (established by the first valid calibration dataset). Mismatched scans SHALL produce an error and disable fusion for the affected dataset.

#### Scenario: Mismatched scan dimensions

- **WHEN** a user loads a PCD whose structured dimensions differ from the session reference
- **THEN** the system shows an error and prevents fusion until the invalid scan is removed
