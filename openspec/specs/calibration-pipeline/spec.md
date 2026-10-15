# calibration-pipeline Specification

## Purpose
Orchestrates manual initial guess and NID extrinsic optimization with compile-time extensible providers that declare multi-dataset support.

## Requirements

### Requirement: Compile-time provider registration

Initial-guess providers and optimizers SHALL be registered at compile time via traits (no runtime plugins). Each provider SHALL declare whether it supports multiple datasets in one solve.

#### Scenario: Multi-dataset NID

- **WHEN** multiple calibration datasets are loaded and NID is selected
- **THEN** the UI allows optimization because NID declares `supports_multi_dataset = true`

### Requirement: Manual initial guess v1

The v1 initial-guess provider SHALL estimate a shared `T_camera_lidar` from per-dataset 3D–2D correspondences using the manual workflow and UI defined in `initial-alignment` (pose seed for ≥3 pairs: not implemented; see `initial-alignment`). ALIKED/LightGlue and target-based methods are out of scope for v1 and SHALL be added as separate initial-alignment methods later.

#### Scenario: Per-dataset correspondence

- **WHEN** the user completes a correspondence pair on calibration dataset A
- **THEN** the pair is stored on dataset A only and contributes to the shared extrinsic when the user runs compute initial alignment

#### Scenario: Delegation to initial-alignment UI

- **WHEN** the user works on the Initial alignment tab
- **THEN** correspondence picking, table editing, and compute initial alignment follow the `initial-alignment` spec

### Requirement: Calibrate tab optimization

NID optimization SHALL be started only from the **Calibrate** tab. The UI SHALL expose session-global `vlcal::VisualCameraCalibrationParams` on that tab (bound to `SessionData::calibration_params` and passed through `AlignmentInput` unchanged into `VisualCameraCalibration::calibrate`). The optimizer SHALL run on all **ready** calibration datasets jointly, sharing one `T_camera_lidar`. Not-ready datasets SHALL be skipped with a visible warning rather than blocking ready datasets from optimizing.

#### Scenario: Partial readiness

- **WHEN** two datasets exist and only one is ready
- **THEN** Run alignment uses the ready dataset and warns that the other was skipped

### Requirement: NID optimization parameters (Calibrate tab)

The **Calibrate** tab SHALL expose every user-relevant field of `vlcal::VisualCameraCalibrationParams` that affects NID behavior. Controls SHALL be grouped (outer loop, inner solver, NID cost, registration type, view culling) with tooltips referencing what each field does. Values SHALL persist in `session.json` with session load/save.

The following mapping SHALL be verified in code review (and documented in `design.md`) by tracing from UI → `session_.calibration_params` → `run_alignment` / `VisualCameraCalibration`:

| Parameter | vlcal usage (reference) |
|-----------|-------------------------|
| `max_outer_iterations` | Outer loop in `VisualCameraCalibration::calibrate` |
| `max_inner_iterations` | Nelder–Mead `max_iterations`; Ceres BFGS `max_num_iterations` |
| `num_threads` | OpenMP `num_threads` in Nelder–Mead cost sum only |
| `delta_trans_thresh`, `delta_rot_thresh` | Outer-loop convergence test in `calibrate` |
| `disable_z_buffer_culling` | `ViewCullingParams::enable_depth_buffer_culling` (inverted) |
| `nid_bins` | `NIDCostParams::bins` / `NIDCost` histogram bins |
| `registration_type` | `NID_BFGS` vs `NID_NELDER_MEAD` branch |
| `nelder_mead_init_step` | Nelder–Mead `init_step` (ignored when BFGS selected) |
| `nelder_mead_convergence_criteria` | Nelder–Mead `convergence_var_thresh` (ignored when BFGS selected) |

`on_progress` is wired by the app (live viewer overlays and RGB); it is not a user-facing Calibrate control.

Before marking implementation complete, the team SHALL confirm each exposed control changes the value passed into vlcal (no dead bindings). Known vlcal gaps (e.g. `num_threads` not applied on the BFGS path) SHALL be called out in UI tooltips or design notes rather than silently exposing a no-op.

#### Scenario: Registration type

- **WHEN** the user selects Nelder–Mead on Calibrate and runs alignment
- **THEN** `registration_type` is `NID_NELDER_MEAD` and Nelder–Mead-specific fields affect the solve

#### Scenario: Session round-trip

- **WHEN** the user changes NID bins and max outer iterations on Calibrate, saves the session, and reloads
- **THEN** the Calibrate controls show the saved values and the next Run alignment uses them

#### Scenario: Parameter audit

- **WHEN** a new `VisualCameraCalibrationParams` field is added upstream in vlcal
- **THEN** the change either adds a Calibrate control with trace documentation or explicitly defers it in `design.md` with rationale

### Requirement: NID optimization v1

The v1 optimizer SHALL run vlcal NID (BFGS or Nelder-Mead) on all ready calibration datasets sharing one `T_camera_lidar`, respecting the global range mask on structured input used to produce each dataset’s cloud and each dataset’s image optimization mask (see `image-optimization-mask`) on that dataset’s camera image during the NID cost. Image masks SHALL apply for both `NID_BFGS` and `NID_NELDER_MEAD` registration types. Datasets without an image mask SHALL use all image pixels in the cost.

#### Scenario: Multi-dataset optimize

- **WHEN** the user runs alignment with two calibration datasets
- **THEN** NID optimizes a single `T_camera_lidar` using both datasets jointly

#### Scenario: Masked pixels excluded from cost

- **WHEN** the user runs alignment and a ready dataset has an image mask with excluded pixels
- **THEN** those pixels do not contribute to the NID mutual-information cost for that dataset

#### Scenario: Nelder-Mead with image mask

- **WHEN** the user selects Nelder–Mead registration and runs alignment with image masks defined
- **THEN** excluded pixels are ignored in the Nelder–Mead cost evaluation the same as in the BFGS path

### Requirement: Live optimization feedback

During NID, the system SHALL stream progress including the current `T_camera_lidar` so the 3D viewer updates the camera-lidar overlay each iteration.

#### Scenario: Progress callback

- **WHEN** the optimizer reports a new iteration
- **THEN** the viewer refreshes projected RGB colors for the current extrinsic

### Requirement: Scene overlays and picking during optimize

While NID optimization runs, correspondence picking SHALL be disabled. Calibration scene overlays on Calibrate SHALL remain non-interactive and the camera triad and frustum SHALL update with each optimizer extrinsic (see `pose-gizmo`).

#### Scenario: No user interference

- **WHEN** optimization is in progress
- **THEN** correspondence picking does not accept input

### Requirement: In-app control tooltips

Primary workflow controls in the calibration UI SHALL expose short tooltips (Qt `ToolTip` on hover, or equivalent) explaining what each control does and when it matters. Tooltips SHALL use plain language, not internal type names alone.

At minimum, tooltips SHALL cover: scan time (time unit, time origin, scan duration), fusion actions and fusion parameters, NID / Calibrate optimization parameters, blend slider, session load/save, and run alignment/fusion buttons.

#### Scenario: Scan time unit

- **WHEN** the user hovers the time-unit control in the Scan time section
- **THEN** a tooltip explains that it selects the unit of per-point timestamps in the PCD (seconds vs nanoseconds) used for motion compensation during fusion

#### Scenario: Scan time origin

- **WHEN** the user hovers the time-origin control
- **THEN** a tooltip explains FirstPoint (re-zero at the first stored point) vs ScanStart (times already relative to sweep/rotation start; use when gaps omit points but timestamps stay valid; requires scan duration for deskew)

#### Scenario: Scan duration

- **WHEN** the user hovers the scan duration field
- **THEN** a tooltip explains it is the full sweep period in seconds, required when time origin is ScanStart, used to normalize point times for deskewing

### Requirement: Camera intrinsics input

Calibration SHALL load camera intrinsics from OpenCV YAML via `cv::FileStorage` (not PyYAML).

#### Scenario: Load intrinsics

- **WHEN** the user provides a camera intrinsics YAML file
- **THEN** the system parses it with OpenCV and uses it for projection and NID
