# Tasks

## 1. vlcal image mask support

- [x] 1.1 Add optional per-image mask to `VisualLiDARData` and NID cost (BFGS + Nelder–Mead) in `vlcal_align`, publish a new package version, and verify a minimal C++ test or vlcal example shows masked pixels change the reported cost
- [x] 1.2 Bump `vlcal_align` requirement in `conanfile.py` and verify `bash scripts/build-app.sh` succeeds against the new package

## 2. Core session and alignment

- [x] 2.1 Add `ImageMaskMeta` to `DatasetEntry`, serialize/deserialize `image_mask` in `session.cpp`, and extend `clc_core_tests` session round-trip to cover empty and populated image mask metadata
- [x] 2.2 Extend `AlignmentInput` with per-dataset image mask buffers, pass masks into `VisualLiDARData` in `calibration_service.cpp`, and verify with a unit test or harness that alignment input rejects dimension mismatch
- [x] 2.3 Wire `AppController` alignment build to load per-dataset masks (from memory or path) and surface a clear error when a saved mask file is missing at run time

## 3. Shared mask editor primitives

- [x] 3.1 Extract shared polygon interaction logic from `RangeImageItem` for reuse and verify existing mask editor behavior unchanged (`clc_app` tests or manual UAT checklist item for lidar mask)
- [x] 3.2 Implement `ImageMaskItem` + `ImageMaskEditorWindow.qml` with dataset image preview and mask overlay, and verify polygon keep/remove updates the per-dataset raster via `AppController`

## 4. Datasets tab I/O and persistence

- [x] 4.1 Add Datasets tab controls (load/save/browse/clear, path display, Edit image mask…) for the selected calibration dataset and verify save writes `image_mask_<id>.png` plus updates `session.json` on Save session
- [x] 4.2 Verify session load restores per-dataset image mask paths and editor shows the correct overlay for the active dataset

## 5. End-to-end verification

- [x] 5.1 Run `bash scripts/build-app.sh`, launch `clc_app`, author an image mask excluding a large region, run NID (BFGS then Nelder–Mead) on a ready dataset, and confirm optimization completes and masked region does not drive visible misalignment in the cost-driven overlay (smoke test per `docs/UAT-checklist.md` if extended)
- [x] 5.2 Update `docs/UAT-checklist.md` with image mask editor and NID steps and verify the new bullets match the implemented UI labels

## Workflow follow-up

- Archive the change with `openspec archive` after review and sync specs to `openspec/specs/`.
