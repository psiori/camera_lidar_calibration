## Purpose

Provides interactive 3D visualization of lidar point clouds with PCL-style navigation, depth-buffer picking, and live scalar/RGB blending for calibration inspection.

## ADDED Requirements

### Requirement: Large point cloud rendering

The 3D viewer SHALL render point clouds of up to approximately 20 million points with interactive frame rates suitable for calibration workflows.

#### Scenario: Dense fused cloud

- **WHEN** a fused cloud of up to 20M points is displayed
- **THEN** the user can navigate and change blend weight without regenerating point positions

### Requirement: Scalar colormap display

The viewer SHALL color points by a selectable scalar channel (intensity, x, y, or z) using a JET colormap with user-settable min and max bounds.

#### Scenario: Switch to Z channel

- **WHEN** the user selects the Z scalar channel
- **THEN** point colors update from the Z coordinate colormap without reloading geometry

### Requirement: Camera RGB blend

Once a camera extrinsic and image are available, the viewer SHALL blend projected camera RGB with the scalar colormap using a user-controlled weight. Changing the blend weight SHALL update rendering only, not geometry.

#### Scenario: Blend slider

- **WHEN** the user moves the blend slider from 0 to 1
- **THEN** point colors interpolate from colormap-only to camera-RGB-only without recomputing positions

### Requirement: PCL trackball navigation

View navigation SHALL follow PCL `pcd_viewer` conventions: left-drag rotate, middle-drag or Shift+left-drag pan, right-drag dolly, wheel dolly, Cmd+drag (macOS) or Ctrl+drag spin around view axis, `r` reset camera, `f` fly-to-point at cursor.

#### Scenario: Trackball rotate

- **WHEN** the user left-drags without modifiers
- **THEN** the view rotates around the focal point in trackball fashion

#### Scenario: Fly to point

- **WHEN** the user presses `f` with the cursor over the cloud
- **THEN** the camera animates to focus on the depth-picked point near the cursor

### Requirement: Point size control

The viewer SHALL support increasing and decreasing rendered point size via `+` and `-` keys.

#### Scenario: Increase point size

- **WHEN** the user presses `+`
- **THEN** rendered point diameter increases

### Requirement: Depth-buffer point picking

3D point picking SHALL use depth-buffer readback and unprojection, not spatial search structures. Picking SHALL account for `devicePixelRatio` on Retina displays.

#### Scenario: Shift-click pick

- **WHEN** the user Shift+left-clicks on the cloud
- **THEN** the system returns the 3D coordinates of the nearest visible point at that screen location

### Requirement: Live optimization preview

During NID optimization, the viewer SHALL update projected RGB colors as `T_camera_lidar` changes each iteration without rebuilding position buffers.

#### Scenario: Optimize preview

- **WHEN** the optimizer reports a new extrinsic
- **THEN** the overlay colors refresh to reflect the updated projection
