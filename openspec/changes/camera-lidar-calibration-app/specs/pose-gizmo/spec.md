## Purpose

Shows the current camera pose in the lidar frame (`T_camera_lidar`) as an in-scene 3D gizmo on **Initial alignment** and **Calibrate**, using the camera image coordinate convention. Full ImGuizmo-style manipulation (as in Iridescence) is the target for a later release on Initial alignment; v1 is display-only on both tabs.

## ADDED Requirements

### Requirement: Initial alignment and Calibrate tabs

The pose gizmo SHALL be drawn on the **Initial alignment** and **Calibrate** top-level tabs when the 3D viewer is shown. It SHALL NOT appear on Datasets or Inspection.

#### Scenario: Gizmo on Calibrate

- **WHEN** the user is on the Calibrate tab with a ready dataset and 3D preview visible
- **THEN** the camera pose gizmo is displayed at the current `T_camera_lidar`

#### Scenario: No gizmo on Datasets

- **WHEN** the user is on the Datasets tab
- **THEN** the 3D view does not show the camera pose gizmo

### Requirement: Camera frame at gizmo origin

The gizmo SHALL be placed at the current camera pose with axes aligned to the **camera image frame**: **+X** along image width (right in the image), **+Y** along image height (down in the image), and **+Z** along the optical axis (forward from the camera into the scene). This matches the convention used for projection and vlcal alignment.

#### Scenario: Axis semantics

- **WHEN** the gizmo is visible
- **THEN** the colored axes at the gizmo origin represent camera X (width), Y (down), and Z (optical axis) in the lidar/world frame

### Requirement: v1 display-only

For v1, the gizmo SHALL be **non-interactive**: it SHALL reflect `T_camera_lidar` but SHALL NOT accept pointer input for translation or rotation. View trackball navigation and Shift+click correspondence picking SHALL behave as if the gizmo were not hittable.

#### Scenario: Drag does not move pose

- **WHEN** the user drags on the gizmo mesh on Initial alignment in v1
- **THEN** the view navigates or picks correspondences as usual and `T_camera_lidar` does not change from the drag alone

### Requirement: Sync with current registration

The displayed gizmo pose SHALL track the session working extrinsic `T_camera_lidar`. It SHALL update when the extrinsic changes from session load, compute initial alignment, NID optimization on the Calibrate tab, or any other pipeline step that updates `T_camera_lidar`.

#### Scenario: After compute initial alignment

- **WHEN** the user runs compute initial alignment and switches to or remains on Initial alignment
- **THEN** the gizmo origin and orientation match the new `T_camera_lidar`

#### Scenario: Live update during NID

- **WHEN** NID is running on the Calibrate tab and the optimizer reports a new `T_camera_lidar`
- **THEN** the gizmo on that tab updates to the current extrinsic each iteration

#### Scenario: After NID

- **WHEN** NID completes on Calibrate
- **THEN** the gizmo on Calibrate and Initial alignment reflects the optimized `T_camera_lidar` whenever those tabs are shown

### Requirement: Separate from view camera

Displaying the gizmo SHALL NOT move or couple to the view navigation camera. The trackball camera remains independent of gizmo pose in v1 and in future interaction modes.

#### Scenario: Navigate while gizmo visible

- **WHEN** the user rotates the view on Initial alignment
- **THEN** the gizmo stays fixed at `T_camera_lidar` in world space

### Requirement: Interactive manipulation target (post-v1)

In a future release, the gizmo on **Initial alignment** SHALL implement ImGuizmo-equivalent interaction (as used in Iridescence): draggable translation on camera axes and rotation about all three camera axes via handles/rings, updating `T_camera_lidar` without moving the view camera. When interaction is enabled, gizmo hits SHALL take priority over view navigation for that gesture; gizmo manipulation SHALL be disabled while NID runs. The Calibrate tab gizmo remains display-only. This behavior is **not** required for v1.

#### Scenario: Future translate handle

- **WHEN** interaction is implemented and the user drags a translation axis
- **THEN** `T_camera_lidar` updates and projected RGB overlay reflects the new pose
