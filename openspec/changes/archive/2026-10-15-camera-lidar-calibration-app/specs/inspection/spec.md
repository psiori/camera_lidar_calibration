## Purpose

Lets users verify calibration results on held-out data and export colored point clouds without re-running optimization.

## ADDED Requirements

### Requirement: Inspection datasets

Users SHALL load inspection (test) datasets separate from calibration datasets. Inspection data MAY undergo fusion and mask application but SHALL NOT run NID unless the user explicitly promotes data to the calibration set.

#### Scenario: Test set overlay

- **WHEN** the user loads an inspection dataset after calibration
- **THEN** the viewer overlays the cloud with the saved `T_camera_lidar` and global mask applied to structured inputs

### Requirement: Inspect calibration inputs

After calibration, users SHALL inspect original calibration datasets with the optimized extrinsic using the same blend and colormap controls as during optimization.

#### Scenario: Review calibration set

- **WHEN** calibration completes
- **THEN** the user can browse each calibration dataset with live blend and colormap controls

### Requirement: Export colored PCD

The system SHALL export point clouds to PCD with xyz, intensity, and rgb fields reflecting the current extrinsic and blend settings at export time.

#### Scenario: Export with RGB

- **WHEN** the user exports a colored cloud
- **THEN** the PCD contains intensity and rgb channels baked at the current `T_camera_lidar`

### Requirement: Global mask on inspection

The global structured mask SHALL apply to inspection dataset scans before any fusion, same as calibration inputs.

#### Scenario: Masked inspection fusion

- **WHEN** the user fuses inspection scans
- **THEN** the global mask is applied to each structured scan first
