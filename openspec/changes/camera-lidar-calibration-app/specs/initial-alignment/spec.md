## Purpose

Provides a unified compile-time extensible workflow to estimate an initial camera–lidar extrinsic before NID optimization, starting with manual 3D–2D correspondence picking and leaving room for future automatic methods (for example feature matching).

## ADDED Requirements

### Requirement: Compile-time initial-alignment methods

The application SHALL register initial-alignment methods at compile time (no runtime plugins). Each method SHALL declare whether it supports solving from multiple calibration datasets in one run. The v1 method SHALL be manual correspondence picking; automatic feature-based methods are out of scope for v1 but SHALL use the same result type and session update path when added.

#### Scenario: Method registry

- **WHEN** the user opens the Initial alignment tab
- **THEN** they can select among methods compiled into the application (v1: manual correspondences only)

### Requirement: Per-method UI capabilities

Each initial-alignment method SHALL declare which UI surfaces it needs (for example image view, 3D view, correspondence editor, in-scene pose seed). The shell SHALL show or hide those surfaces based on the selected method without hard-coding a single layout for all future methods.

#### Scenario: Manual method surfaces

- **WHEN** manual correspondence picking is selected
- **THEN** the user sees the tri-pane alignment workspace (image, correspondence panel, 3D viewer) described in `app-shell`

### Requirement: Tri-pane alignment workspace

On the **Initial alignment** top-level tab, the main work area SHALL use three adjacent panes: **camera image on the left**, **correspondence controls and table in the center**, and **3D point cloud on the right**. The user SHALL be able to resize panes with splitters.

#### Scenario: Enter initial alignment

- **WHEN** the user selects the Initial alignment tab
- **THEN** the image, correspondence table, and 3D viewer are visible side by side

### Requirement: Ready active dataset for picking

The 3D viewer SHALL display the **active** calibration dataset’s ready cloud (fused or single-scan output). The user SHALL NOT switch image or PCD paths from this tab. Datasets that are not **ready** SHALL be selectable only where allowed by `app-shell` (greyed out on this tab with tooltip). Correspondence picking SHALL be enabled only when the active dataset is ready.

#### Scenario: Unfused dataset disabled

- **WHEN** the active dataset is not ready
- **THEN** Shift+pick and image picking are disabled and the selector entry explains that fusion is required

### Requirement: Zoomable camera image with reticle

The left pane SHALL display the active calibration dataset’s camera image in a pan- and zoom-capable view. A reticle (crosshair) SHALL follow the pointer over the image to support accurate pixel selection. Picking SHALL map display coordinates to native image pixel coordinates accounting for letterboxing or uniform scale.

#### Scenario: Zoom and pick

- **WHEN** the user zooms and pans the image and clicks to complete a correspondence
- **THEN** the stored image coordinates match the pixel under the reticle in the source image

### Requirement: Correspondence-based initial solve

Manual correspondence solving SHALL estimate the initial transform from picked 2D–3D pairs only (vlcal `PoseEstimation` with trusted correspondences). Passing the session working `T_camera_lidar` as a nonlinear pose seed for solves with three or more pairs is **not** implemented in v1 (reserved for post-v1 gizmo interaction). A **display-only** camera pose gizmo (see `pose-gizmo`) SHALL visualize `T_camera_lidar` on Initial alignment and Calibrate; v1 does not allow editing pose via the gizmo.

#### Scenario: Compute uses all pairs

- **WHEN** the user has added correspondences on one or more datasets and runs compute initial alignment with valid intrinsics
- **THEN** the solver receives all correspondence pairs, writes `T_camera_lidar_initial` and the working extrinsic, and the gizmo display updates to match

### Requirement: Active dataset image and cloud stay paired

On Datasets, Initial alignment, and Calibrate tabs, the camera image preview, 3D point cloud, and camera RGB projection SHALL always reflect the **same** active calibration dataset. Switching datasets or tabs SHALL reload both image and cloud for that dataset; the application SHALL NOT show one dataset’s image with another dataset’s cloud.

#### Scenario: Switch calibration dataset

- **WHEN** the user selects a different calibration dataset on any of those tabs
- **THEN** the image pane and 3D viewer update to that dataset’s image and cloud (fused or preview) together

### Requirement: Camera pose gizmo on Initial alignment

The Initial alignment 3D pane SHALL show the camera pose gizmo at `T_camera_lidar` when a ready active dataset is displayed. The gizmo SHALL use the camera frame convention and v1 display-only rules in `pose-gizmo`.

#### Scenario: Visual feedback

- **WHEN** the user opens Initial alignment with a ready dataset and loaded intrinsics
- **THEN** the gizmo is visible at the current registration pose while they pick correspondences

### Requirement: Correspondence picking flow

For the manual method on the active calibration dataset, the user SHALL pick a 3D point in the right-hand viewer (Shift+left click per `point-cloud-viewer`), then pick the matching 2D location on the image. The system SHALL show markers for pending and committed pairs in both the image and 3D views. Picking and marker editing SHALL be disabled while fusion or NID workers are running.

#### Scenario: Complete one pair

- **WHEN** the user Shift+clicks a 3D point and then clicks the matching image location on the current dataset
- **THEN** a correspondence row is added with markers in both panes

#### Scenario: Blocked during optimize

- **WHEN** NID optimization is in progress
- **THEN** correspondence picking and table edits do not accept input

### Requirement: Correspondence table across datasets

The center pane SHALL list correspondences from **all** calibration datasets in one table. Rows belonging to the **currently selected** calibration dataset SHALL be editable (pick, re-pick, delete). Rows from other datasets SHALL be visible but read-only and SHALL be visually distinct (for example muted style or dataset label). Deleting an editable row SHALL remove both markers from the image and 3D views and remove the row from the session.

#### Scenario: Read-only other dataset

- **WHEN** the table contains pairs from dataset A and B and the user has dataset A active
- **THEN** rows for dataset B are visible but cannot be edited or deleted until the user switches active dataset to B

#### Scenario: Delete pair

- **WHEN** the user deletes a row for the active dataset
- **THEN** markers for that pair disappear from image and 3D and the correspondence is removed from session data

### Requirement: Selected row re-pick

The user SHALL be able to select a table row for the active dataset. With a row selected, the next 3D or image pick SHALL update that row’s corresponding endpoint instead of creating a new row. Selection SHALL highlight markers for that pair in both panes.

#### Scenario: Update image side

- **WHEN** a row is selected and the user clicks a new image location
- **THEN** the image coordinates for that correspondence update and markers move accordingly

### Requirement: Compute initial alignment on all datasets

The user SHALL run **Compute initial alignment** when the manual method’s minimum pair count is satisfied **across all calibration datasets collectively** (v1: at least three pairs total with valid intrinsics). The solver SHALL use every stored correspondence on every calibration dataset to estimate a **single** shared `T_camera_lidar`. On success, the application SHALL update the working extrinsic and refresh projected RGB on the **active** dataset’s image and cloud so the user can inspect the result with the 3D viewer blend control.

#### Scenario: Multi-dataset solve

- **WHEN** dataset A has two pairs and dataset B has one pair and intrinsics are loaded
- **THEN** compute initial alignment is enabled and produces one extrinsic using all three pairs

#### Scenario: Inspect with blend

- **WHEN** compute initial alignment succeeds
- **THEN** the user can adjust the blend slider on the 3D viewer to compare lidar scalar coloring with projected camera RGB

### Requirement: Session persistence

Per-dataset correspondences and the computed initial extrinsic SHALL be saved and loaded with the session per `session` spec. Switching the active calibration dataset SHALL load that dataset’s image in the left pane while keeping the unified table content.

#### Scenario: Reload session

- **WHEN** the user loads a session with saved correspondences on multiple datasets
- **THEN** the table shows all pairs and markers reflect stored coordinates after datasets and clouds are available
