## Status note

Implementation was completed **before** this OpenSpec change was opened (posthumous documentation). All tasks below are marked done to reflect the tree as of 2026-10-14.

## 1. clc_linalg foundation

- [x] 1.1 Add `linalg/` INTERFACE target `clc_linalg` and `include/clc/linalg.hpp` (Vec2f/3f/3d, Mat4f/4d, layout docs, lookAt/perspective, rigid inverse, quat I/O)
- [x] 1.2 Wire `add_subdirectory(linalg)` from root `CMakeLists.txt`

## 2. clc_core boundary

- [x] 2.1 Replace `Eigen::Isometry3d` with `Mat4d` in `session.hpp`, `calibration_service.hpp`, `point_color_model.hpp`, `nid_cloud_prep.hpp`, `range_image.hpp` (`Vec2f` polygons)
- [x] 2.2 Add `eigen_adapters.{hpp,cpp}`; link `Eigen3::Eigen` PRIVATE on `clc_core`
- [x] 2.3 Update `session.cpp` pose JSON via `mat4dToQuaternionTranslation` / `mat4dFromQuaternionTranslation`
- [x] 2.4 Update `calibration_service.cpp`, `point_color_model.cpp`, `nid_cloud_prep.cpp`, `range_image.cpp`
- [x] 2.5 Fix `clc_core_tests` pose setup (Eigen only for fusion cloud fixture points)

## 3. psiviz

- [x] 3.1 Remove `find_package(Eigen3)` and PUBLIC Eigen link; depend on `clc_linalg`
- [x] 3.2 Migrate headers: `trackball_camera`, `pose_gizmo` (`Mat4f` pose), `depth_picker`, `scene_shapes`, `point_cloud_item`, `point_cloud_renderer`
- [x] 3.3 Migrate sources: camera, gizmo, picker, shapes, renderer, viewer_input, `point_cloud_item.cpp`
- [x] 3.4 Update `psiviz/docs/API.md` (gizmo pose type)
- [x] 3.5 Update `trackball_camera_test.cpp` and `depth_picker_test.cpp`

## 4. clc_app

- [x] 4.1 `app_controller.hpp`: `std::optional<clc::Vec3d>` for pending 3D pick (no Eigen)
- [x] 4.2 `app_controller.cpp`: `mat4dToPoseMatrix`, `Mat4d` session/gizmo sync; `Vec3d` color model points; `mat4dFromColumnMajor` for vlcal/fusion Eigen poses in `.cpp` only
- [x] 4.3 Mask polygon `Vec2f` vertices (`push_back` / `.x`/`.y` accessors)

## 5. Verification

- [x] 5.1 Build `clc_core`, `psiviz`, `clc_app` (dev tree `CLC_DEV_BUILD=ON`)
- [x] 5.2 Run `clc_core_tests`, `psiviz_trackball_test`, `ctest`
- [x] 5.3 Smoke-launch `clc_app` (few seconds)

## 6. Optional follow-ups (not done)

- [ ] 6.1 Move vlcal `Isometry3d` → `Mat4d` conversion out of `app_controller.cpp` into `clc_core` seam helper
- [ ] 6.2 Record incremental compile benchmark (Eigen in headers before vs after) in BUILD.md
