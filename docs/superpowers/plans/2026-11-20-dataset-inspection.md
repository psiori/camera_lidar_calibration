# Dataset Inspection Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task by task. The user has requested no new tests; update existing tests only where changed behavior requires it.

**Goal:** Configure, fuse and inspect either kind of dataset in one place, with settings and masks applied to the correct data.

**Architecture:** Retain separate calibration and inspection lists, with an explicit selected dataset on the Datasets tab and a remembered calibration selection for calibration-only tabs. Store scan masks by width and height in the session; keep the image optimization mask global. Validate each dataset's scans independently and preserve detailed errors through to the UI.

**Tech stack:** C++17, Qt Quick/QML, CMake, existing session JSON and PNG mask storage.

**Spec:** `docs/superpowers/specs/2026-11-20-dataset-inspection-design.md`

## Global constraints

- Images across a session have identical dimensions; scan shapes may differ between datasets, but scans within a dataset must match.
- One scan mask per shape; one global image optimization mask. A new scan shape keeps all points by default.
- Preserve existing sessions, calibration behavior, inspection export and existing masks.
- Do not add new tests; update existing tests only when required by changed functionality.

## Review focus

- Legacy inspection datasets must retain session-level fusion and timing settings on load.
- A legacy scan mask must survive migration and match only its original shape.
- Removing the last dataset of a shape must not remove its mask.
- A mismatched scan or image must show the file and actual and expected dimensions.
- Switching dataset types must not use another dataset's settings or include inspection data in calibration.

## Task 1: Session data and migration

**Files:** `clc_core/include/clc_core/session.hpp`, `clc_core/src/session.cpp`, `clc_core/src/dataset_util.cpp` and related existing session tests as needed.

**Interface:** Extend `clc::SessionData` with scan mask metadata keyed by scan width and height, preserving the existing global image mask. Each `clc::DatasetEntry` continues to own its `fusion_params` and `scan_time`.

- [ ] Persist and reload every shape's mask operations, image path and dimensions; keep entries when datasets are removed.
- [ ] Migrate the old global scan mask to its stored shape without resizing or losing its PNG or operations. Preserve existing image-mask metadata.
- [ ] Load legacy inspection fusion and timing from session-level values where the old format used them; write and reload inspection-specific values thereafter.
- [ ] Update existing session checks as needed and run the existing core checks using the configured build described in `BUILD.md`.

## Task 2: Selection and dataset-specific settings

**Files:** `clc_app/app_controller.hpp`, `clc_app/app_controller.cpp`, `clc_core/src/dataset_util.cpp` and related existing checks as needed.

**Interface:** Expose inspection dataset IDs, selected inspection index, add/remove/select actions and the selected dataset kind to QML. Preserve the selected calibration index independently.

- [ ] Route `activeDatasetEntry()` and all dataset settings, including scan time and fusion parameters, through the selected dataset rather than the main tab or session defaults.
- [ ] Support adding, selecting and removing multiple inspection datasets. Reuse existing input browsing and identity conventions; keep calibration selection when inspection is active.
- [ ] Restore the remembered calibration dataset on entering Initial alignment or Calibrate, and keep inspection datasets out of calibration inputs.
- [ ] Verify switching between datasets with different scan times leaves each value intact, including after saving and loading.

## Task 3: Geometry, masks and actionable errors

**Files:** `clc_core/src/scan_geometry.cpp`, `clc_core/include/clc_core/scan_geometry.hpp`, `clc_app/app_controller.cpp`, `clc_app/app_controller.hpp`, existing mask editor QML as needed.

**Interface:** Use `clc::scan_pcd_source` and its `PcdDirectoryScan` result for dataset-local dimensions and error details; resolve the active scan mask from its dimensions in `SessionData`.

- [ ] Validate scan dimensions within each dataset instead of against a shared session scan reference. Preserve detailed file and dimension errors when fusion is blocked.
- [ ] Validate camera image dimensions across both dataset lists; report the file and actual and expected sizes on mismatch.
- [ ] Load, edit, save and apply the mask associated with the selected scan shape. Reuse the existing mask for matching shapes; initialize new shapes with all points kept. Keep the image optimization mask global.
- [ ] Check the existing mask and fusion paths for single scans and dataset switches, and update existing checks where changed behavior requires it.

## Task 4: Dataset UI and inspection workflow

**Files:** `clc_app/Main.qml`, `clc_app/AddDatasetDialog.qml`, `clc_app/ImageMaskEditorWindow.qml`, other existing mask editor QML as needed.

**Interface:** Consume the selection and inspection actions from Task 2 and the geometry and mask information from Task 3.

- [ ] Show calibration and inspection rows on Datasets, with add/select/remove controls and readiness details. Show the selected dataset's inputs and settings.
- [ ] Remove the Inspection tab and show colored PCD export when an inspection dataset is selected; retain the shared viewer and fusion controls.
- [ ] Label the image mask as global and the scan mask with its dimensions and sharing scope. Show detailed validation errors next to the affected dataset and fusion action.
- [ ] Manually exercise add, select, fuse, mask edit, export and switching to calibration-only tabs. Check both matching and differing scan shapes and image-size errors.

## Task 5: Integration verification

**Files:** `BUILD.md`, existing checks and `docs/UAT-checklist.md` as references; update existing checks only when needed.

- [ ] Build the app using the configured build instructions in `BUILD.md` and run the existing relevant checks; record any unavailable dependencies.
- [ ] Open an older session, save and reload it, and verify inspection settings and masks survive. Check independent settings, shared masks by shape, image mask sharing, actionable errors and inspection export.
- [ ] Review the final changes against the approved design and the five review-focus cases above.
