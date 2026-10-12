## Purpose

Documents the `clc_app` main-window structure: four workflow tabs, shared calibration-dataset selector, session chrome, mask popout, and viewer overlay. Complements `session`, `range-image-mask`, `initial-alignment`, and `calibration-pipeline`.

## ADDED Requirements

### Requirement: Four-tab main window

The application SHALL expose exactly four top-level tabs: **Datasets**, **Initial alignment**, **Calibrate**, and **Inspection**. The app SHALL NOT use a separate Loading / preprocessing tab or a nested calibration section strip.

#### Scenario: Workflow tabs

- **WHEN** the user launches the app
- **THEN** they can switch among Datasets, Initial alignment, Calibrate, and Inspection with visually distinct segmented tab chrome

### Requirement: Visually distinct tab chrome

All top-level tabs SHALL use a segmented control (or equivalent) where the **selected** tab has clearly distinct styling from inactive tabs.

#### Scenario: Active tab obvious

- **WHEN** the user is on Initial alignment
- **THEN** that segment appears selected and the other three appear inactive

### Requirement: Session bar outside tabs

Session directory, **Load**, and **Save** SHALL appear on the main window below the tab content and SHALL remain available on all tabs.

#### Scenario: Save from Inspection tab

- **WHEN** the user is on the Inspection tab and activates Save
- **THEN** the session is written without switching to Datasets

### Requirement: Shared calibration dataset selector

On **Datasets**, **Initial alignment**, and **Calibrate**, the UI SHALL show the same calibration-dataset selector (tabs or equivalent) reflecting every calibration dataset in the session. The user SHALL change the **active** dataset from this selector on any of those three tabs. **Inspection** SHALL NOT use the calibration dataset list (inspection datasets remain separate per `session`).

#### Scenario: Active dataset follows user

- **WHEN** the user selects dataset B on the Datasets tab and switches to Initial alignment
- **THEN** dataset B remains the active calibration dataset until they select another

### Requirement: Add datasets only on Datasets tab

The user SHALL add or remove calibration datasets only on the **Datasets** tab. Initial alignment and Calibrate SHALL NOT expose controls to create new calibration datasets.

#### Scenario: No add on align tab

- **WHEN** the user is on Initial alignment
- **THEN** there is no Add dataset action; only the shared selector and alignment tools

### Requirement: Add calibration dataset dialog

**Add…** on the Datasets tab SHALL open a small modal dialog (not a chain of native file pickers). The dialog SHALL list exactly what must be chosen before the dataset is created:

1. **Camera image** — one image file (PNG/JPEG/etc.) paired with this dataset.
2. **LiDAR scans** — either a **directory** of structured `.pcd` files (multi-scan fusion) or a **single** `.pcd` file (single-scan / passthrough).
3. **Camera intrinsics** — session-global OpenCV `FileStorage` YAML; required before NID but MAY be set here when adding a dataset (or left unchanged if the session already has intrinsics).

Each row SHALL have a short hint label, a read-only path field, and a **Browse…** button scoped to the correct picker (image file, PCD file or directory, intrinsics YAML). **OK** SHALL validate all required paths, run geometry / duplicate-id checks, then create the dataset and close the dialog. **Cancel** SHALL discard pending paths. The main window SHALL NOT open an image file picker immediately when the user clicks **Add…**.

#### Scenario: Clear add flow

- **WHEN** the user clicks **Add…** on Datasets
- **THEN** they see the three rows with hints and can browse each input before confirming

#### Scenario: Intrinsics on add

- **WHEN** the user confirms Add with a valid intrinsics YAML selected
- **THEN** the session `intrinsics_path` and loaded camera model are updated (same as **Load intrinsics** on Datasets)

#### Scenario: Cancel add

- **WHEN** the user closes the dialog with Cancel
- **THEN** no new calibration dataset is added and the dataset list is unchanged

### Requirement: Datasets tab content

The **Datasets** tab SHALL provide per-dataset configuration for the **active** dataset: image path and PCD directory (set when the dataset is created), per-dataset scan-time and fusion parameters (new datasets copy values from the first calibration dataset), fuse action, global mask file shortcuts, and **Edit mask…**. Intrinsics SHALL remain session-global on this tab. After a dataset is added, its image and PCD paths SHALL NOT be editable; the user SHALL delete the dataset and add a new one to change paths (path editing may be added later).

#### Scenario: Immutable paths

- **WHEN** a calibration dataset already exists with baked paths
- **THEN** the UI does not offer path text fields for that dataset; delete is required to replace inputs

### Requirement: Single-scan fusion and scan-time UI

On the **Datasets** tab, when the **active** calibration dataset has exactly **one** structured PCD in its catalog (single `.pcd` file at add time, or a directory that resolved to one scan), the UI SHALL **disable and visually grey out** the **Scan time** controls and the multi-scan **Fusion** controls (`num_threads`, `max_correspondence_distance`). Scan-time fields SHALL NOT apply to single-scan passthrough (no cross-scan deskew). **`voxel_resolution`** and **Run fusion** SHALL remain enabled so the user can run single-scan passthrough / downsampling per `lidar-fusion`. Disabled groups SHALL show a tooltip explaining that only output voxel size and **Run fusion** apply for a single cloud.

#### Scenario: One PCD selected

- **WHEN** the active dataset’s catalog contains one PCD
- **THEN** time unit, time origin, scan duration, fusion threads, and max correspondence are greyed out and non-interactive

#### Scenario: Multi-scan directory

- **WHEN** the active dataset’s catalog contains two or more PCDs
- **THEN** fusion and scan-time controls are enabled (subject to geometry validation and worker busy state)

### Requirement: Initial alignment tri-pane layout

The **Initial alignment** tab SHALL use a horizontal **SplitView** with three panes: **camera image (left)**, **correspondence method, compute actions, and correspondence table (center)**, and **3D point cloud with viewer overlay (right)**. Pane widths SHALL be user-adjustable. The image pane SHALL support pan and zoom for labeling. The in-scene camera pose gizmo (v1 display-only) SHALL appear on this tab when a ready dataset is shown (see `pose-gizmo`).

#### Scenario: Side-by-side picking

- **WHEN** the user is on Initial alignment with a ready active dataset
- **THEN** they can pick on the image and in the 3D viewer without leaving the tab

### Requirement: Calibrate tab scope

The **Calibrate** tab SHALL expose session-global NID / visual calibration parameters and **Run alignment** (automatic optimization). It SHALL show a 3D preview with viewer overlay and the **display-only** camera pose gizmo (see `pose-gizmo`), updated as `T_camera_lidar` changes during and after optimization. It SHALL NOT host the correspondence editor.

#### Scenario: Automatic calibration

- **WHEN** the user runs alignment from Calibrate
- **THEN** NID runs without manual pose gizmo interaction on that tab while the gizmo reflects each reported extrinsic

### Requirement: Dataset readiness in selector

Calibration datasets that are not **ready** (per `session`) SHALL appear in the shared selector but SHALL be disabled or greyed out on **Initial alignment** and **Calibrate**, with a tooltip explaining that fusion is required (or a single-scan passthrough must be completed). **Datasets** tab SHALL remain usable to fuse or configure disabled datasets.

#### Scenario: Unfused dataset tooltip

- **WHEN** the user hovers a greyed-out dataset on Initial alignment
- **THEN** a tooltip states that the dataset must be fused (or single-scan ready) before labeling or optimization

### Requirement: Viewer overlay controls

On tabs that show the 3D viewer (at minimum **Datasets**, **Initial alignment**, and **Calibrate**), scalar channel, colormap min/max, and blend SHALL appear in an overlay on the point cloud viewer, not in a separate control column unless the tab layout requires both.

#### Scenario: Blend on viewer

- **WHEN** the user adjusts blend on Initial alignment
- **THEN** the control is on or over the 3D view

### Requirement: Mask popout and global mask I/O

**Edit mask…** SHALL open a dedicated resizable mask editor window. **Load mask**, **Browse mask…**, **Save mask**, and **Clear mask** SHALL live on the **Datasets** tab (global mask group). The mask editor SHALL include a calibration-dataset selector so the user can preview the range image from any dataset when drawing the shared mask (see `range-image-mask`).

#### Scenario: Edit mask from Datasets

- **WHEN** the user clicks Edit mask… on Datasets
- **THEN** the popout opens and can switch which dataset’s structured scan is displayed for authoring

### Requirement: Inspection tab scope

The Inspection tab SHALL provide inspection-oriented dataset paths and fuse action in a scrollable layout. This requirement does not add new inspection export or multi-dataset features beyond other capability specs.

#### Scenario: Inspection controls

- **WHEN** the user switches to the Inspection tab
- **THEN** they can set inspection dataset paths and run fuse without the calibration dataset selector
