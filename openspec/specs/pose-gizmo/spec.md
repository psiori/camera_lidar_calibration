# pose-gizmo Specification

## Purpose
Draws non-interactive calibration scene overlays in the 3D viewer: a camera coordinate triad and perspective frustum at the camera pose in the lidar frame, and a lidar coordinate triad at the displayed scan origin. Session `T_camera_lidar` maps **lidar points into the camera frame** (same convention as projection and NID); the viewer places the camera triad and frustum using **`T_lidar_camera = inverse(T_camera_lidar)`**. There is no in-scene pose manipulation.

## Requirements

### Requirement: Tab visibility for overlays

The **lidar** RGB triad SHALL appear on **Datasets**, **Initial alignment**, and **Calibrate** whenever the 3D viewer shows a loaded active calibration dataset. It SHALL NOT appear on **Inspection**.

The **camera** RGB triad and **camera frustum** SHALL appear on those same tabs only when the session marks an extrinsic as available (`extrinsic_available`: after compute initial alignment, NID, or load of a calibrated session). Until then, the default identity `T_camera_lidar` SHALL NOT be visualized. The camera triad SHALL be placed at **`T_lidar_camera`**. The frustum additionally requires camera intrinsics plus the active image size.

#### Scenario: No camera overlays before extrinsic

- **WHEN** the user adds a new dataset with intrinsics loaded but has not run compute initial alignment or NID
- **THEN** the lidar triad is visible when the cloud is shown and the camera triad and frustum are omitted

#### Scenario: Frustum on Datasets

- **WHEN** the user is on the Datasets tab with a calibration dataset whose point cloud is visible in the 3D preview and intrinsics are loaded
- **THEN** the white camera frustum is drawn at the camera pose in the lidar frame

#### Scenario: Triads on Datasets

- **WHEN** the user is on the Datasets tab with a calibration dataset whose point cloud is visible in the 3D preview
- **THEN** the camera and lidar RGB triads are drawn at `T_lidar_camera` and the cloud origin respectively

#### Scenario: Full overlays on calibration tabs

- **WHEN** the user is on Datasets, Initial alignment, or Calibrate with a loaded active dataset cloud visible
- **THEN** the camera triad and lidar triad are visible; the camera frustum is visible when intrinsics are available (frustum omitted if intrinsics are missing)

### Requirement: Camera coordinate triad

The viewer SHALL draw a **camera** coordinate triad at **`T_lidar_camera`**. Each axis SHALL be **1 m** long from the camera origin, with standard RGB coloring (**+X** red, **+Y** green, **+Z** blue). Axes SHALL align to the **camera image frame**: **+X** along image width (right in the image), **+Y** along image height (down in the image), and **+Z** along the optical axis (forward from the camera into the scene).

#### Scenario: Axis semantics

- **WHEN** the camera triad is visible
- **THEN** the three 1 m segments at the camera origin represent camera X (width), Y (down), and Z (optical axis) expressed in the lidar frame

### Requirement: Lidar coordinate triad

The viewer SHALL draw a **lidar** coordinate triad at the origin of the displayed point cloud (sensor / scan frame). Each axis SHALL be **1 m** long with the same RGB axis coloring as the camera triad. Axis directions SHALL follow the native axes of the structured scan / fused cloud data.

#### Scenario: Lidar frame at scan origin

- **WHEN** a calibration dataset cloud is shown on Initial alignment or Calibrate
- **THEN** a 1 m RGB triad is drawn at the cloud origin indicating lidar +X, +Y, and +Z

### Requirement: Camera perspective frustum

The viewer SHALL draw a **short** wireframe perspective frustum in **white** at **`T_lidar_camera`**, using the active dataset’s loaded camera intrinsics (focal lengths and principal point / image size). Corners SHALL be built in the camera image frame (+X right, +Y down, +Z optical) and transformed into the lidar frame with **`T_lidar_camera`**. The frustum SHALL extend **2 m** along **+Z** from the camera origin unless a shorter depth is needed for scene scale. It SHALL update when intrinsics or `T_camera_lidar` change. If intrinsics are missing, the frustum MAY be omitted while the camera triad still reflects pose on alignment tabs.

#### Scenario: Frustum matches intrinsics

- **WHEN** intrinsics are loaded and the frustum is shown
- **THEN** the white frustum apex is at the camera position in the lidar frame and its opening angles match the calibrated image field of view at 2 m depth along the optical axis

### Requirement: Non-interactive overlays

All calibration scene overlays SHALL be **non-interactive**: they SHALL NOT accept pointer input for translation or rotation and SHALL NOT participate in hit-testing ahead of view navigation or correspondence picking.

#### Scenario: Drag does not move pose

- **WHEN** the user drags over overlay geometry
- **THEN** the view navigates or picks correspondences as usual and `T_camera_lidar` does not change from the drag alone

### Requirement: Sync with current registration

The camera triad and frustum SHALL track session `T_camera_lidar` via **`T_lidar_camera`**. They SHALL update when the extrinsic changes from session load, compute initial alignment, NID on Calibrate, or any other pipeline step that updates `T_camera_lidar`. The lidar triad SHALL remain at the cloud origin.

#### Scenario: After compute initial alignment

- **WHEN** the user runs compute initial alignment
- **THEN** the camera triad and frustum match the new `T_camera_lidar`

#### Scenario: Live update during NID

- **WHEN** NID is running on the Calibrate tab and the optimizer reports a new `T_camera_lidar`
- **THEN** the camera triad and frustum update each iteration

### Requirement: Separate from view camera

Drawing overlays SHALL NOT move or couple to the view navigation camera.

#### Scenario: Navigate while overlays visible

- **WHEN** the user rotates the view
- **THEN** the camera triad and frustum stay fixed at `T_lidar_camera` in the lidar frame and the lidar triad stays at the cloud origin
