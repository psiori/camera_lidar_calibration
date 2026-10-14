## Purpose

Defines shared fixed-size math types and compile-time boundaries so CLC libraries expose poses and vectors without pulling Eigen into their public headers.

## ADDED Requirements

### Requirement: Column-major 4x4 layout

The project SHALL provide `clc::Mat4f` and `clc::Mat4d` types whose in-memory layout is column-major 4×4, indexable as `m[col * 4 + row]`, compatible with Eigen `Matrix4` and typical OpenGL/QRhi uniform uploads.

#### Scenario: Matrix element addressing

- **WHEN** code sets `M(2, 1)` on a `clc::Mat4d`
- **THEN** the value is stored at index `1 * 4 + 2` in the underlying `m[16]` array

### Requirement: Session pose API uses Mat4d

`SessionData` and calibration service result types SHALL represent `T_camera_lidar` (and initial pose) as `clc::Mat4d`, not as Eigen transform types in public headers.

#### Scenario: Load session pose

- **WHEN** `session.json` contains a 7-element pose array (translation + quaternion)
- **THEN** `load_session` populates `SessionData::T_camera_lidar` as a rigid `Mat4d` without requiring Eigen in the including translation unit

### Requirement: Viewer pose API uses Mat4f

The point cloud viewer and pose gizmo public API SHALL use `clc::Mat4f` for extrinsic pose and `clc::Vec3f` for 3D vectors in headers.

#### Scenario: QML pose matrix bridge

- **WHEN** QML supplies 16 floats in column-major order to `setPoseMatrix`
- **THEN** the gizmo stores them as `clc::Mat4f` and `poseMatrix` returns the same 16 values

### Requirement: No Eigen in CLC public headers

Public headers under `psiviz/include`, `clc_core/include`, and `clc_app` SHALL NOT `#include` Eigen headers.

#### Scenario: App controller header compile

- **WHEN** a translation unit includes only `app_controller.hpp` and `clc_core` public headers
- **THEN** the preprocessor does not pull in `Eigen/Core` or `Eigen/Geometry` from those headers

### Requirement: Eigen at vlcal boundaries only in implementation

Conversion between `clc::Mat4d` and vlcal Eigen types SHALL occur only in `clc_core` implementation files (or app `.cpp` callbacks), not in public headers.

#### Scenario: Run alignment

- **WHEN** `run_alignment` is called with `AlignmentInput::init_T_camera_lidar` as `Mat4d`
- **THEN** calibration invokes vlcal with an Eigen isometry built inside `calibration_service.cpp`

### Requirement: psiviz does not link Eigen publicly

The `psiviz` CMake target SHALL NOT declare a public dependency on `Eigen3::Eigen`.

#### Scenario: Consumer links psiviz

- **WHEN** a target links `psiviz` without linking Eigen explicitly
- **THEN** compile succeeds for TUs that include only psiviz and `clc_linalg` headers
