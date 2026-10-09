## Purpose

Defines persistent session state for camera-LiDAR calibration: one rigid extrinsic, calibration and inspection datasets, and global mask metadata.

## ADDED Requirements

### Requirement: Single extrinsic per session

The session SHALL store exactly one `T_camera_lidar` (camera pose in lidar frame) shared by all calibration and inspection datasets in that session.

#### Scenario: Multiple datasets same extrinsic

- **WHEN** the user adds a second calibration dataset
- **THEN** the system applies the same `T_camera_lidar` to both datasets without per-dataset extrinsic overrides

### Requirement: Calibration and inspection datasets

The session SHALL distinguish calibration datasets (used for initial guess and NID optimization) from inspection datasets (overlay and export only, no optimization unless explicitly promoted).

#### Scenario: Inspection dataset overlay

- **WHEN** the user loads an inspection dataset after calibration
- **THEN** the system applies the saved `T_camera_lidar` for visualization without running NID

### Requirement: Per-dataset image and cloud pairing

Each dataset SHALL contain exactly one image and one point cloud (fused output or single scan).

#### Scenario: Fused cloud pairing

- **WHEN** fusion completes for a set of masked scans
- **THEN** the resulting unstructured cloud is paired with one user-selected image as a single dataset entry

### Requirement: Session persistence

The session SHALL be saveable and loadable to a directory including intrinsics, extrinsics, mask path, fusion parameters, scan-time settings, dataset paths, and per-dataset correspondences.

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

All structured input scans in a session MUST share the same `width` and `height` as the first loaded scan. Mismatched scans SHALL produce an error and disable fusion.

#### Scenario: Mismatched scan dimensions

- **WHEN** a user loads a PCD whose structured dimensions differ from the session reference
- **THEN** the system shows an error and prevents fusion until the invalid scan is removed
