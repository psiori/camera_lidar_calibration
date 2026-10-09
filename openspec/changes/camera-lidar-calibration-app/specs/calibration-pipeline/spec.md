## Purpose

Orchestrates manual initial guess and NID extrinsic optimization with compile-time extensible providers that declare multi-dataset support.

## ADDED Requirements

### Requirement: Compile-time provider registration

Initial-guess providers and optimizers SHALL be registered at compile time via traits (no runtime plugins). Each provider SHALL declare whether it supports multiple datasets in one solve.

#### Scenario: Multi-dataset NID

- **WHEN** multiple calibration datasets are loaded and NID is selected
- **THEN** the UI allows optimization because NID declares `supports_multi_dataset = true`

### Requirement: Manual initial guess v1

The v1 initial-guess provider SHALL combine in-scene pose gizmo manipulation with per-dataset Shift+click 3D–2D correspondences (solvePnP optional intermediate). ALIKED/LightGlue and target-based methods are out of scope for v1.

#### Scenario: Per-dataset correspondence

- **WHEN** the user Shift+clicks a 3D point and picks the matching image location for dataset A
- **THEN** the correspondence is stored on dataset A only and contributes to the shared extrinsic estimate

### Requirement: NID optimization v1

The v1 optimizer SHALL run vlcal NID (BFGS or Nelder-Mead) on all calibration datasets sharing one `T_camera_lidar`, respecting the global range mask on structured input.

#### Scenario: Multi-dataset optimize

- **WHEN** the user runs alignment with two calibration datasets
- **THEN** NID optimizes a single `T_camera_lidar` using both datasets jointly

### Requirement: Live optimization feedback

During NID, the system SHALL stream progress including the current `T_camera_lidar` so the 3D viewer updates the camera-lidar overlay each iteration.

#### Scenario: Progress callback

- **WHEN** the optimizer reports a new iteration
- **THEN** the viewer refreshes projected RGB colors for the current extrinsic

### Requirement: Gizmo disabled during optimize

While optimization runs, the pose gizmo SHALL be disabled (see `pose-gizmo` spec). Correspondence picking SHALL also be disabled during optimization.

#### Scenario: No user interference

- **WHEN** optimization is in progress
- **THEN** gizmo and correspondence picking do not accept input

### Requirement: Camera intrinsics input

Calibration SHALL load camera intrinsics from OpenCV YAML via `cv::FileStorage` (not PyYAML).

#### Scenario: Load intrinsics

- **WHEN** the user provides a camera intrinsics YAML file
- **THEN** the system parses it with OpenCV and uses it for projection and NID
