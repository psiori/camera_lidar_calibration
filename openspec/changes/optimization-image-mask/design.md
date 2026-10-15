# Design

## Context

Today `clc::run_alignment` builds `vlcal::VisualLiDARData` from each dataset’s `cv::Mat` image and fused cloud with no per-pixel image mask. The global lidar range mask (`GlobalMaskMeta`, `mask_ops_`, `RangeImageItem`) applies only when converting structured PCDs to scans for fusion via `to_lidar_scan_masked`. Mask raster utilities already live in `clc_core/range_image.hpp` (`build_mask_from_ops`, PNG I/O, 255=keep encoding).

NID registration type is selected via `vlcal::VisualCameraCalibrationParams::registration_type` (`NID_BFGS` vs `NID_NELDER_MEAD`). The vlcal author’s guidance is to gate NID histogram / residual accumulation on a per-image mask inside the NID cost functor (and the parallel cost path used by Nelder–Mead).

## Goals / Non-Goals

**Goals:**

- Per-dataset optional image masks, edited with the same polygon UX as the lidar mask editor.
- Masks persisted per dataset in the session directory and referenced from `session.json`.
- Masks honored in vlcal for both BFGS and Nelder–Mead during `Run alignment`.
- Reuse `clc_core` mask rasterization and PNG helpers; reduce duplication between `RangeImageItem` and a new image mask item.

**Non-Goals:**

- Masking inspection-only datasets or changing RGB overlay rendering during live NID preview (may still show full image unless we later tie overlay to mask).
- Automatic mask generation (segmentation, sky detection).
- Migrating old `session.json` files that lack new keys (pre-stable policy: new required fields on save; load fails if schema incomplete).
- Changing global lidar mask semantics or dimensions.

## Decisions

### 1. Session model: per-dataset `ImageMaskMeta`

Add to `DatasetEntry` (mirror `GlobalMaskMeta`):

```cpp
struct ImageMaskMeta {
  std::string path;
  int width = 0;
  int height = 0;
};
```

Runtime editing state in `AppController` (per active dataset index): `image_mask_ops_`, `image_mask_pixels_`, `image_mask_editor_open_`. Serialize under each dataset in `session.json` as `image_mask: { path, width, height }`.

**Alternative:** Session-global map `id -> path` at top level. Rejected: keeps mask co-located with dataset entry and matches fusion/correspondence patterns.

### 2. vlcal integration

**Preferred:** Extend the `vlcal_align` Conan package so `VisualLiDARData` accepts an optional `cv::Mat` or `std::vector<uint8_t>` mask (same size as image, 255=use pixel in NID). Thread mask into:

- Ceres `NIDCost` (BFGS): skip masked-out pixels when accumulating joint histogram / MI.
- Nelder–Mead cost callback: same gating when summing per-pixel contributions.

Bump `vlcal_align` version in `conanfile.py` after upstream change; document the required API in package release notes.

**Alternative:** Vendor a forked vlcal only in this repo. Rejected unless Conan publish is blocked; prefer versioned package.

**Alternative:** Pre-zero image regions in `cv::Mat` before vlcal. Rejected: does not remove points from cost the same way as mask gating and can distort intensity statistics.

`AlignmentInput` gains `std::vector<std::vector<std::uint8_t>> image_masks` (empty inner vector = no mask for that dataset index). `run_alignment` passes masks when constructing each `VisualLiDARData`.

### 3. UI: image mask editor

- New `ImageMaskEditorWindow.qml` (modal window, dataset selector when multiple calibration datasets).
- New `ImageMaskItem` (QQuickPaintedItem): copy interaction from `RangeImageItem` but map clicks to **image** native pixels (no Y-stretch unless we add optional preview stretch; default 1:1 with image aspect).
- Extract shared polygon painting / vertex undo / `polygonCommitted` signal into a small C++ helper or base class used by `RangeImageItem` and `ImageMaskItem` to avoid divergent hit-testing bugs.

Datasets tab: new group **Image optimization mask** (distinct from **Global lidar mask**): path label, Load/Save/Browse/Clear, **Edit image mask…**.

Tooltips: clarify this mask affects NID only, not fusion.

### 4. Default mask filename

On save from UI: `<session_dir>/image_mask_<dataset_id>.png` (sanitize `id` for filesystem). Store absolute or session-relative path consistently with `global_mask.png` handling today.

### 5. Alignment pipeline

In `AppController::buildAlignmentInput` (or equivalent): for each ready dataset, load image dimensions from disk, ensure `image_mask_pixels` match or leave empty; push into `AlignmentInput`. If on-disk path is set but file missing at run time, fail alignment with a clear error (do not silently run unmasked).

## Risks / Trade-offs

- **[vlcal API not yet available]** → Block on `vlcal_align` release; stub with compile-time flag only if needed for parallel UI work.
- **[Large images, many datasets]** → Masks are one byte per pixel; memory is negligible vs images/clouds.
- **[User forgets mask on one dataset]** → Joint solve mixes masked and unmasked datasets; document in tooltip; no automatic warning in v1.
- **[Session schema break]** → Pre-stable: all new saves include `image_mask` object per dataset (empty path when unset); document that older saves must be re-saved or hand-edited.

## Migration Plan

1. Ship vlcal with mask support and bump Conan pin.
2. Ship clc_core session + alignment wiring.
3. Ship UI editor and Datasets controls.
4. Users re-save sessions to pick up new JSON fields; no automatic migration.

## Open Questions

- Confirm exact `VisualLiDARData` constructor / setter in the vlcal version we pin (name and mask value convention).
- Whether live NID `on_progress` RGB overlay should respect image mask for visual feedback (defer unless cheap).
