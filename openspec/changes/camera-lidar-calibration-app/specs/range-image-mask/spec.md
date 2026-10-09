## Purpose

Lets users author a global exclusion mask on structured lidar range/intensity images before calibration, to remove rig-mounted static objects from all scans.

## ADDED Requirements

### Requirement: Structured range image display

The range image view SHALL display a structured lidar scan as a 2D `width × height` image where each pixel encodes range or intensity. This view SHALL NOT use camera intrinsics or extrinsics.

#### Scenario: Range channel display

- **WHEN** the user selects the range channel on a loaded structured PCD
- **THEN** each pixel shows that beam's range value colormapped with JET and user-set min/max bounds

### Requirement: Display Y-stretch

The range image view SHALL provide a user-adjustable vertical stretch factor so squished native aspect ratios are easier to label.

#### Scenario: Stretch for labeling

- **WHEN** the user increases the Y-stretch factor
- **THEN** the displayed image is stretched vertically without changing underlying mask topology

### Requirement: Polygon mask authoring

The user SHALL draw one or more polygons on the range image with a keep/remove-inside toggle. Polygon vertices SHALL be stored in native structured image coordinates (inverse stretch applied on input).

#### Scenario: Remove inside polygon

- **WHEN** the user draws a polygon with mode "remove inside" and confirms
- **THEN** pixels inside the polygon are marked excluded in the global mask

### Requirement: Global mask scope

The mask SHALL be authored once from a reference structured scan and applied identically to every structured PCD before fusion. The mask SHALL NOT apply to post-fusion unstructured clouds.

#### Scenario: Apply mask to batch

- **WHEN** fusion runs on multiple structured scans
- **THEN** each scan is filtered by the global mask before entering the fusion pipeline

### Requirement: Binary mask I/O

The mask SHALL be saveable and loadable as a binary image matching reference `width × height`.

#### Scenario: Save mask

- **WHEN** the user saves the mask to disk
- **THEN** a binary `width × height` image is written that can be reloaded in a later session

### Requirement: Independent colormap bounds

The range image widget SHALL have its own colormap min/max controls, independent of the 3D point cloud viewer.

#### Scenario: Separate bounds

- **WHEN** the user adjusts range-image colormap bounds
- **THEN** the 3D viewer colormap bounds are unchanged
