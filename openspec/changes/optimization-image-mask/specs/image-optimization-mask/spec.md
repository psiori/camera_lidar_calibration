# Spec Delta

## Purpose

Lets users exclude regions of each calibration camera image from NID optimization so static or misleading pixels do not affect the mutual-information cost.

## ADDED Requirements

### Requirement: Per-dataset image mask scope

Each calibration dataset SHALL have its own optional image optimization mask. The mask SHALL be a binary raster with one byte per image pixel matching that dataset’s loaded image width and height. Value 255 SHALL mean the pixel contributes to NID; value 0 SHALL mean the pixel is ignored in the NID cost for that dataset only. The mask SHALL NOT alter fusion, the global lidar range mask, or inspection visualization unless a future change says otherwise.

#### Scenario: Independent masks per dataset

- **WHEN** dataset A and dataset B have different image masks authored
- **THEN** NID applies dataset A’s mask only to A’s image and dataset B’s mask only to B’s image in the joint solve

#### Scenario: No mask defined

- **WHEN** a ready calibration dataset has no image mask file and no in-session raster
- **THEN** all pixels of that dataset’s image are treated as contributing to NID (same as pre-change behavior)

### Requirement: Image mask editor window

The application SHALL provide a dedicated resizable window to author the image mask for the selected calibration dataset, launched from the Datasets tab (for example **Edit image mask…**). The editor SHALL display that dataset’s calibration image, support polygon draw with finish, and **Keep inside** / **Remove inside** (or equivalent) actions matching the lidar mask editor workflow. Polygon vertices SHALL be stored in native image pixel coordinates.

#### Scenario: Open editor for active dataset

- **WHEN** the user opens the image mask editor for a calibration dataset with a valid image path
- **THEN** the window shows that image with the current mask overlay and accepts polygon edits

#### Scenario: Dataset without image

- **WHEN** the user attempts to open the image mask editor for a dataset with no readable image
- **THEN** the system shows an error and does not open an empty editor

### Requirement: Per-dataset image mask I/O

The Datasets tab SHALL expose load, save, browse, and clear actions for the **image optimization mask** of the selected calibration dataset, separate from the global lidar range mask controls. Saved masks SHALL be binary images matching the dataset image dimensions. Default save location SHALL be alongside the session directory using a stable per-dataset filename derived from the dataset id (implementation may use `image_mask_<id>.png`).

#### Scenario: Save image mask

- **WHEN** the user saves the image mask for a dataset
- **THEN** a binary `image_width × image_height` file is written and the session records its path for that dataset

#### Scenario: Load image mask dimension mismatch

- **WHEN** the user loads a mask file whose dimensions do not match the dataset image
- **THEN** load fails with a clear error and the previous mask for that dataset is unchanged

### Requirement: Image mask does not block readiness

Absence of an image mask SHALL NOT make a dataset not-ready. Image mask authoring SHALL be optional for optimization.

#### Scenario: Ready without image mask

- **WHEN** a dataset is otherwise ready for NID
- **THEN** Run alignment is allowed even if no image mask has been created
