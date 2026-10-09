## Purpose

Provides an in-scene 3D gizmo to manipulate the camera pose in the lidar frame (`T_camera_lidar`) for initial guess and manual refinement, separate from view navigation.

## ADDED Requirements

### Requirement: In-scene extrinsic manipulation

The pose gizmo SHALL be rendered as a 3D widget in the scene (not a view-corner widget) that manipulates `T_camera_lidar`, representing where the camera is in the lidar frame.

#### Scenario: Translate camera pose

- **WHEN** the user drags a translation axis on the gizmo
- **THEN** `T_camera_lidar` updates and the camera-lidar overlay reflects the new pose

### Requirement: Translation and rotation

The gizmo SHALL support independent translation (Tx, Ty, Tz) and rotation (Rx, Ry, Rz) about the camera frame axes.

#### Scenario: Rotate about axis

- **WHEN** the user drags a rotation ring aligned with an axis
- **THEN** only rotation about that axis is applied to `T_camera_lidar`

### Requirement: Separate from view camera

Manipulating the pose gizmo SHALL NOT move the view navigation camera. View trackball controls remain available when the gizmo is not capturing input.

#### Scenario: Navigate while gizmo visible

- **WHEN** the user left-drags on empty space (no gizmo hit)
- **THEN** the view camera rotates and `T_camera_lidar` is unchanged

### Requirement: Disabled during optimization

The pose gizmo SHALL be disabled while NID optimization is running. The user SHALL NOT manipulate the extrinsic during optimization.

#### Scenario: Optimize in progress

- **WHEN** NID optimization starts
- **THEN** the gizmo becomes non-interactive until optimization completes or is cancelled

### Requirement: Re-enable after optimization

After optimization completes, the gizmo SHALL be re-enabled so the user can nudge the extrinsic and re-run optimization from the adjusted pose.

#### Scenario: Nudge and re-optimize

- **WHEN** optimization finishes and the user adjusts the gizmo
- **THEN** `T_camera_lidar_initial` or the working extrinsic updates and the user can start a new optimization run

### Requirement: Gizmo input priority

When the gizmo is hit-tested under the cursor, gizmo manipulation SHALL take priority over view navigation for that gesture.

#### Scenario: Drag gizmo handle

- **WHEN** the user presses on a gizmo handle
- **THEN** the gesture manipulates the pose, not the view camera
