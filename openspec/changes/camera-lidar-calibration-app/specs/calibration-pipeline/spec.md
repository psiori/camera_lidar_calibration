## Purpose

Orchestrates manual initial guess and NID extrinsic optimization with compile-time extensible providers that declare multi-dataset support.

## ADDED Requirements

### Requirement: Compile-time provider registration

Initial-guess providers and optimizers SHALL be registered at compile time via traits (no runtime plugins). Each provider SHALL declare whether it supports multiple datasets in one solve.

#### Scenario: Multi-dataset NID

- **WHEN** multiple calibration datasets are loaded and NID is selected
- **THEN** the UI allows optimization because NID declares `supports_multi_dataset = true`

### Requirement: Manual initial guess v1

The v1 initial-guess provider SHALL estimate a shared `T_camera_lidar` from per-dataset 3D–2D correspondences and an in-scene pose seed, using the manual workflow and UI defined in `initial-alignment`. ALIKED/LightGlue and target-based methods are out of scope for v1 and SHALL be added as separate initial-alignment methods later.

#### Scenario: Per-dataset correspondence

- **WHEN** the user completes a correspondence pair on calibration dataset A
- **THEN** the pair is stored on dataset A only and contributes to the shared extrinsic when the user runs compute initial alignment

#### Scenario: Delegation to initial-alignment UI

- **WHEN** the user works on the Initial alignment tab
- **THEN** correspondence picking, table editing, and compute initial alignment follow the `initial-alignment` spec

### Requirement: Calibrate tab optimization

NID optimization SHALL be started only from the **Calibrate** tab. The UI SHALL expose session-global `VisualCameraCalibrationParams` (or equivalent) there. The optimizer SHALL run on all **ready** calibration datasets jointly, sharing one `T_camera_lidar`. Not-ready datasets SHALL be skipped with a visible warning rather than blocking ready datasets from optimizing.

#### Scenario: Partial readiness

- **WHEN** two datasets exist and only one is ready
- **THEN** Run alignment uses the ready dataset and warns that the other was skipped

### Requirement: NID optimization v1

The v1 optimizer SHALL run vlcal NID (BFGS or Nelder-Mead) on all ready calibration datasets sharing one `T_camera_lidar`, respecting the global range mask on structured input used to produce each dataset’s cloud.

#### Scenario: Multi-dataset optimize

- **WHEN** the user runs alignment with two calibration datasets
- **THEN** NID optimizes a single `T_camera_lidar` using both datasets jointly

### Requirement: Live optimization feedback

During NID, the system SHALL stream progress including the current `T_camera_lidar` so the 3D viewer updates the camera-lidar overlay each iteration.

#### Scenario: Progress callback

- **WHEN** the optimizer reports a new iteration
- **THEN** the viewer refreshes projected RGB colors for the current extrinsic

### Requirement: Gizmo and picking during optimize

While NID optimization runs, correspondence picking SHALL be disabled. The pose gizmo on Calibrate SHALL remain display-only and SHALL update with each optimizer extrinsic; on Initial alignment the gizmo is display-only in v1. When interactive gizmo is added post-v1 on Initial alignment, it SHALL be non-interactive during optimization (see `pose-gizmo`).

#### Scenario: No user interference

- **WHEN** optimization is in progress
- **THEN** correspondence picking does not accept input

### Requirement: In-app control tooltips

Primary workflow controls in the calibration UI SHALL expose short tooltips (Qt `ToolTip` on hover, or equivalent) explaining what each control does and when it matters. Tooltips SHALL use plain language, not internal type names alone.

At minimum, tooltips SHALL cover: scan time (time unit, time origin, scan duration), fusion actions and fusion parameters, blend slider, session load/save, and run alignment/fusion buttons.

#### Scenario: Scan time unit

- **WHEN** the user hovers the time-unit control in the Scan time section
- **THEN** a tooltip explains that it selects the unit of per-point timestamps in the PCD (seconds vs nanoseconds) used for motion compensation during fusion

#### Scenario: Scan time origin

- **WHEN** the user hovers the time-origin control
- **THEN** a tooltip explains FirstPoint (zero time at the first point in each scan) vs ScanStart (zero at the scan header stamp, requires scan duration)

#### Scenario: Scan duration

- **WHEN** the user hovers the scan duration field
- **THEN** a tooltip explains it is the full sweep period in seconds, required when time origin is ScanStart, used to normalize point times for deskewing

### Requirement: Camera intrinsics input

Calibration SHALL load camera intrinsics from OpenCV YAML via `cv::FileStorage` (not PyYAML).

#### Scenario: Load intrinsics

- **WHEN** the user provides a camera intrinsics YAML file
- **THEN** the system parses it with OpenCV and uses it for projection and NID
