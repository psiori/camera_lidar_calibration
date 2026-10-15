# Proposal

## Why

NID extrinsic optimization currently uses every pixel of each calibration image when building the visual–LiDAR mutual-information cost. Sky, vehicle body, bloom, and other regions that do not carry useful texture bias the solve and are hard to fix with the existing **global lidar range mask** alone (that mask only culls structured scans before fusion). Per-image exclusion masks during optimization are a standard vlcal workflow; the app needs authoring, persistence, and wiring into both BFGS and Nelder–Mead NID paths.

## What Changes

- Add a **per calibration dataset** binary mask on camera image pixels (`width × height` of that dataset’s image), independent of the session-global lidar range mask.
- Provide an **image mask editor** (dedicated window, polygon keep/remove, preview on the dataset image) reusing the same interaction model as the lidar range mask editor.
- Expose **load, save, browse, and clear** for each dataset’s image mask on the Datasets tab (and editor shortcuts where appropriate); persist mask file paths in `session.json`.
- Pass masks into **vlcal NID** so excluded pixels do not contribute to the NID cost for that dataset, for **both** `NID_BFGS` and `NID_NELDER_MEAD` registration types.
- When no image mask is defined for a dataset, behavior matches today (all image pixels eligible).
- Refactor shared mask polygon / raster logic between lidar and image editors (implementation detail; lidar mask requirements unchanged).

## Capabilities

### New Capabilities

- `image-optimization-mask`: Per-dataset camera image masks, editor UI, I/O, and application during NID optimization.

### Modified Capabilities

- `calibration-pipeline`: NID optimization SHALL respect per-dataset image masks in addition to the global lidar range mask.
- `session`: Session persistence SHALL include per-dataset image mask metadata and round-trip mask file references.

## Impact

- **clc_core**: `DatasetEntry` and session JSON serialization; `AlignmentInput` / `run_alignment` to attach per-image masks to vlcal datasets; possible **vlcal_align** package bump or patch for NID cost masking (BFGS Ceres functor and Nelder–Mead cost sum).
- **clc_app**: New QML window and painted item (or generalized mask surface item); `AppController` state per dataset; Datasets tab controls; alignment build path loads masks before `VisualCameraCalibration::calibrate`.
- **Dependencies**: `vlcal_align/0.1.0` (Conan) likely needs API or internal changes for image masks; coordinate version pin in `conanfile.py`.
- **Specs**: No change to inspection-only workflows unless masks are later reused for overlay (out of scope).
