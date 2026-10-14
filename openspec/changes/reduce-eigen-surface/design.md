## Context

See `proposal.md`. Before this change, `psiviz` linked `Eigen3::Eigen` publicly and `clc_core` exposed `Eigen::Isometry3d` in `session.hpp` and calibration headers. The app and viewer recompiled large portions of Eigen on unrelated edits.

## Goals / Non-Goals

**Goals:**

- No Eigen includes in **public** headers of `psiviz`, `clc_core` (our headers), or `clc_app`.
- Poses in domain APIs as **4×4 column-major** `Mat4d` / `Mat4f` with documented layout.
- Single shared header (`clc/linalg.hpp`) for float (viewer) and double (calibration) math.
- Eigen only in `clc_core` `.cpp` files at vlcal/gtsam/PCL boundaries via `detail::eigen_adapters`.

**Non-Goals:**

- Removing Eigen from the link line or from vlcal/PCL/gtsam translation units.
- Wrapping vlcal types to hide their Eigen usage.
- Replacing Eigen inside fusion or vlcal libraries.

## Decisions

| Decision | Choice | Alternatives considered |
|----------|--------|-------------------------|
| Math types | Own `clc::Mat4d` / `Vec3f` in `linalg/include/clc/linalg.hpp` | Qt `QMatrix4x4` (couples psiviz API to Qt); glm (still heavy headers) |
| Pose representation | Homogeneous 4×4 matrix in APIs | Quaternion + translation (JSON already uses quat; conversion stays in `session.cpp`) |
| psiviz link | `clc_linalg` only, no `find_package(Eigen3)` | Keep Eigen for gizmo only (rejected: headers still dominate compile) |
| clc_core Eigen | `PRIVATE` link + `eigen_adapters.cpp` | PUBLIC link with opaque handles (more boilerplate, same compile win in our headers) |
| vlcal results in app | `mat4dFromColumnMajor(iso.matrix().data())` in `app_controller.cpp` | Public adapter header including Eigen (rejected: exposes Eigen to app includes) |
| Session JSON | Unchanged 7-vector pose | Store 16 matrix entries (rejected: breaks existing sessions) |

### Layout contract

- `Mat4f::m[i]` / `Mat4d::m[i]` with `operator()(row, col) → m[col*4+row]`.
- Compatible with `Eigen::Matrix4f` and OpenGL uniform uploads (`memcpy` 16 floats).

### File map (implemented)

- `linalg/CMakeLists.txt` — INTERFACE library `clc_linalg`
- `clc_core/src/eigen_adapters.{hpp,cpp}` — `toIsometry` / `fromIsometry` / `toEigen(Vec3d)`
- Boundary TUs: `calibration_service.cpp`, `point_color_model.cpp`, `nid_cloud_prep.cpp`, `app_controller.cpp` (vlcal progress only)

## Risks / Trade-offs

- **[Risk] Silent layout misuse** when hand-rolling matrix code → Mitigation: documented `operator()` indexing; GPU paths use `.data()` same as before.
- **[Risk] BREAKING C++ API** for any external user of `clc_core` headers → Mitigation: documented in proposal; JSON/QML unchanged.
- **[Trade-off] Duplicated mini linear algebra** vs Eigen → Acceptable; small header, no expression templates.
- **[Trade-off] App `.cpp` still includes Eigen** for fusion/alignment callbacks → Minimal; headers stay clean.

## Migration Plan

1. Depend on `clc_linalg` from `psiviz` and `clc_core`.
2. Replace `Eigen::Isometry3d` session fields with `Mat4d`; keep `save_pose` / `load_pose` quat logic.
3. Convert psiviz gizmo from `Affine3f` to `Mat4f` with `translation()` / `rotationColumn()` helpers.
4. Run `clc_core_tests`, psiviz tests, build `clc_app`.

Rollback: revert the branch; session JSON on disk remains compatible.

## Open Questions

- Optional: move app-side `mat4dFromColumnMajor(iso.matrix().data())` into `clc_core` to drop `Eigen/Geometry` from `app_controller.cpp` entirely.
- Optional: compile-time benchmark (touch `app_controller.cpp` vs `point_cloud_item.cpp` incremental build) recorded in BUILD.md.
