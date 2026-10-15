#include <cmath>
#include <cstdlib>

#include <clc/linalg.hpp>
#include <psiviz/trackball_camera.hpp>

namespace {

clc::Vec3f eyeFromView(const clc::Mat4f& view) {
  const clc::Vec3f t{view(0, 3), view(1, 3), view(2, 3)};
  return {-(view(0, 0) * t.x + view(1, 0) * t.y + view(2, 0) * t.z),
          -(view(0, 1) * t.x + view(1, 1) * t.y + view(2, 1) * t.z),
          -(view(0, 2) * t.x + view(1, 2) * t.y + view(2, 2) * t.z)};
}

}  // namespace

int main() {
  psiviz::TrackballCamera camera;
  camera.setViewport(800, 600, 1.f);
  camera.fitToBounds({-1.f, -1.f, -1.f}, {1.f, 1.f, 1.f});

  const clc::Vec3f focal = camera.focalPoint();
  if (std::abs(focal.x) > 0.2f || std::abs(focal.y) > 0.2f || std::abs(focal.z) > 0.2f) {
    return EXIT_FAILURE;
  }

  const clc::Vec3f eye0 = eyeFromView(camera.viewMatrix());
  camera.rotate(40.f, 0.f);
  const clc::Vec3f eye1 = eyeFromView(camera.viewMatrix());
  if (clc::length(eye1 - eye0) < 1e-3f) {
    return EXIT_FAILURE;
  }

  camera.fitToBounds({-2.f, -2.f, -2.f}, {2.f, 2.f, 2.f});
  const clc::Mat4f prev = camera.viewMatrix();
  camera.pan(50.f, 0.f);
  const clc::Mat4f after = camera.viewMatrix();
  if (std::abs(prev(0, 3) - after(0, 3)) < 1e-5f && std::abs(prev(1, 3) - after(1, 3)) < 1e-5f) {
    return EXIT_FAILURE;
  }

  camera.fitToBounds({-40.f, -40.f, -40.f}, {40.f, 40.f, 40.f});
  const float near_before_dolly = camera.clipNearPlane();
  camera.dolly(800.f);
  const float near_after_zoom_in = camera.clipNearPlane();
  if (near_after_zoom_in >= near_before_dolly) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
