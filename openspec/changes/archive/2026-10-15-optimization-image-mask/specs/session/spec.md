# Spec Delta

## MODIFIED Requirements

### Requirement: Session persistence

The session SHALL be saveable and loadable to a directory including intrinsics, extrinsics, global mask path, per-dataset fusion and scan-time settings, dataset ids and paths, fused cloud paths, per-dataset correspondences, and per-dataset image optimization mask path and dimensions (see `image-optimization-mask`).

#### Scenario: Round-trip session contents

- **WHEN** the user saves the session and later loads the same session directory
- **THEN** intrinsics, extrinsics, global lidar mask reference, per-dataset image mask references, dataset paths, fusion settings, fused cloud paths, and correspondences are restored

### Requirement: Session load and save from UI

The UI SHALL expose session directory, Load, and Save on the main window.

#### Scenario: Load session from UI

- **WHEN** the user chooses a session directory and activates Load
- **THEN** `session.json` is read and the UI shows restored dataset paths, fusion/scan-time settings, intrinsics (if saved), global and per-dataset image mask references when saved, and `T_camera_lidar` (camera triad and frustum overlays reflect the restored pose)

#### Scenario: Save session from UI

- **WHEN** the user activates Save for the current session directory
- **THEN** `session.json` reflects the current session state (paths, parameters, extrinsic, global lidar mask reference, and per-dataset image mask references when applicable)
