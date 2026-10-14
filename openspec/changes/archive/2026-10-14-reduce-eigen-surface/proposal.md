## Why

Including Eigen in public headers (`psiviz`, `clc_core`, `clc_app`) forces every downstream translation unit to parse heavy template code and slows incremental builds. vlcal, PCL, and gtsam still require Eigen at link time, but our own APIs do not need to expose it.

## What Changes

- Introduce **`clc_linalg`**: header-only `clc::Vec*`, `clc::Mat4f` / `clc::Mat4d` with documented column-major layout (Eigen/OpenGL compatible).
- **psiviz**: drop public `Eigen3::Eigen` dependency; poses and camera math use `clc::Mat4f` / `Vec3f`.
- **clc_core**: session poses and service APIs use `clc::Mat4d`; Eigen linked **PRIVATE**; vlcal conversions in `.cpp` adapters only.
- **clc_app**: no Eigen in headers; `.cpp` boundary copies vlcal `Isometry3d` matrices via `mat4dFromColumnMajor`.
- **BREAKING** (C++ API): `SessionData::T_camera_lidar` and related types change from `Eigen::Isometry3d` to `clc::Mat4d`; `PointColorModel` and polygon vertices use `clc` vector types.
- Document post-hoc: implementation landed before this change was opened (see `tasks.md`).

## Capabilities

### New Capabilities

- `linalg-public-types`: Shared fixed-size math types, layout contract, and Eigen isolation rules for CLC libraries.

### Modified Capabilities

(none — no archived `openspec/specs/` yet; app behavior unchanged, only C++ surface types)

## Impact

- **New**: `linalg/` CMake INTERFACE target, `include/clc/linalg.hpp`
- **psiviz**: CMake, all public headers and sources; `docs/API.md` gizmo pose type
- **clc_core**: public headers, `session.cpp`, `calibration_service.cpp`, `point_color_model.cpp`, `nid_cloud_prep.cpp`, `eigen_adapters.*`
- **clc_app**: `app_controller.hpp` / `.cpp`
- **Tests**: `psiviz` trackball/depth_picker tests, `clc_core_tests` pose setup
- **Unchanged**: JSON session pose format (tx,ty,tz + quaternion); QML 16-float pose matrices; vlcal/PCL headers still pull Eigen into TUs that include them
