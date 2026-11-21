# Dataset setup and inspection

## Intent

Make calibration and inspection datasets equally easy to configure and fuse. Allow different scan settings and scan shapes across datasets, and explain any condition that blocks fusion.

## Approaches

- **Selected:** Two dataset rows on the Datasets tab with a common setup panel. Selecting an inspection dataset makes its inspection actions available there. This keeps setup and switching in one place.
- Keep a separate Inspection tab with duplicate setup controls. This preserves the existing separation but makes switching and maintaining consistent controls harder.

## Interface and behavior

- Show calibration and inspection datasets in separate rows on the Datasets tab, with add, select and remove actions for both. Clearly identify the selected dataset and show useful readiness information.
- Remove the Inspection tab. On selecting an inspection dataset, show its inputs, fusion settings, mask controls, viewer and colored PCD export in the Datasets tab. Inspection datasets never participate in calibration.
- When entering a calibration-only tab, use the last selected calibration dataset. Selecting an inspection dataset must not lose that selection.
- Store dataset-specific settings, including scan timing and fusion settings, with each dataset. Switching datasets shows and uses its own values.
- All images in a session must have the same dimensions. Report a mismatch with the actual and expected dimensions and identify the dataset or file.
- Scan shapes may differ between datasets. Scans within one dataset must have compatible shapes. Report the affected file and its actual and expected dimensions when validation blocks fusion; show other available validation errors as well.
- The image optimization mask is global. All datasets use the same mask, and edits apply everywhere.
- Maintain one scan mask for each scan shape. Datasets with the same shape use that same mask, so edits are shared among them. A new shape starts unmasked, with all points kept; a dataset with a known shape automatically uses the existing mask for that shape. Make the sharing clear in the editor and dataset controls. Keep masks when the last dataset of a shape is removed.

## Existing sessions and boundaries

- Preserve existing calibration and inspection inputs and settings when loading older sessions. Legacy inspection fusion used session-level settings, so initialize its dataset settings from those values. Carry existing scan-mask operations, image and dimensions forward for the matching shape; preserve the global image mask and legacy image-mask metadata. Save the new per-shape masks and per-dataset settings so they survive reload.
- Keep existing calibration and inspection capabilities, including colored PCD export. This change does not redesign calibration or fusion algorithms.
- Avoid silent mask resizing, silent loss of existing settings, and generic errors that leave the user unable to tell what to fix.

## Implementation choices

The implementer should choose the session format and migration details that preserve existing data, and adapt the existing Qt controls and validation flow. The interface should make clear which dataset and scan shape an edit affects.
