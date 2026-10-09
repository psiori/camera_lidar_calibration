## Purpose

Documents the agreed `clc_app` main-window structure from UI planning (two-tab shell, mask popout, session chrome, layout polish). Complements `range-image-mask` (mask behavior) and `session` (persistence).

## ADDED Requirements

### Requirement: Two-tab main window

The application SHALL expose exactly two top-level tabs: **Calibration** and **Inspection**. The app SHALL NOT use a separate Loading / preprocessing tab.

#### Scenario: Calibration vs inspection

- **WHEN** the user launches the app
- **THEN** they see Calibration and Inspection tabs and can switch between them without a third preprocessing tab

### Requirement: Visually distinct tab chrome

Calibration and Inspection SHALL use a segmented control (or equivalent) where the **selected** mode has clearly distinct styling (background, border, or bold label) from the inactive mode.

#### Scenario: Active tab obvious

- **WHEN** the user is on Calibration
- **THEN** the Calibration segment appears selected and Inspection appears inactive

### Requirement: Session bar outside tabs

Session directory, **Load**, and **Save** SHALL appear on the main window below the tab content and SHALL remain available on both tabs.

#### Scenario: Save from Inspection tab

- **WHEN** the user is on the Inspection tab and activates Save
- **THEN** the session is written without switching to Calibration

### Requirement: Calibration split and scroll

On the Calibration tab, a horizontal **SplitView** SHALL default to approximately **50 / 50** width between the control column and the 3D viewer (user-adjustable). The control column SHALL live inside a vertical **ScrollView** with collapsible sections; **Dataset** SHALL be expanded by default and other sections collapsed by default.

#### Scenario: Short window

- **WHEN** the window height is smaller than the full control stack
- **THEN** the user can scroll the left column without clipping controls

### Requirement: Viewer overlay controls

On the Calibration tab, scalar channel, colormap min/max, and blend SHALL appear in an overlay on the point cloud viewer, not in the left control column.

#### Scenario: Blend on viewer

- **WHEN** the user adjusts blend
- **THEN** the control is on or over the 3D view while dataset and fusion controls remain in the scroll column

### Requirement: Mask popout vs main-window mask I/O

On the Calibration tab, **Edit mask…** SHALL open a dedicated resizable window for range-image mask authoring. **Load mask**, **Browse mask…**, **Save mask**, and **Clear mask** SHALL remain on the main window Global mask group.

#### Scenario: Edit mask in popout

- **WHEN** the user clicks Edit mask…
- **THEN** a separate window opens for drawing polygons and adjusting display (channel, Y-stretch, colormap); mask file actions stay on the main window

#### Scenario: No inline range image on main window

- **WHEN** the user views the Calibration tab left column
- **THEN** the structured range image editor is not embedded there (only the popout and mask file shortcuts)

### Requirement: Inspection tab scope (UI pass)

The Inspection tab SHALL provide inspection-oriented dataset paths and fuse action in a scrollable collapsible layout. This requirement does not add new inspection export or multi-dataset features beyond other capability specs.
