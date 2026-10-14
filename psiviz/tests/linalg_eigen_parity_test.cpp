// Compare clc:: lookAt / perspective / multiply to Eigen for psiviz camera matrices.

#include <cmath>
#include <cstdlib>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <clc/linalg.hpp>
#include <psiviz/trackball_camera.hpp>

namespace {

bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

bool matricesNear(const clc::Mat4f& a, const Eigen::Matrix4f& b, float eps = 1e-4f) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!near(a(r, c), b(r, c), eps)) {
        std::fprintf(stderr, "mismatch at (%d,%d): clc=%g eigen=%g\n", r, c, a(r, c), b(r, c));
        return false;
      }
    }
  }
  return true;
}

Eigen::Matrix4f eigenViewLikeTrackball(const Eigen::Vector3f& eye, const Eigen::Vector3f& center,
                                       const Eigen::Vector3f& up) {
  Eigen::Vector3f f = center - eye;
  if (f.squaredNorm() < 1e-12f) {
    f = Eigen::Vector3f(0.f, 0.f, -1.f);
  } else {
    f.normalize();
  }
  Eigen::Vector3f s = f.cross(up);
  if (s.squaredNorm() < 1e-12f) {
    s = f.cross(Eigen::Vector3f(0.f, 1.f, 0.f));
  }
  s.normalize();
  const Eigen::Vector3f u = s.cross(f);
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view.block<1, 3>(0, 0) = s.transpose();
  view.block<1, 3>(1, 0) = u.transpose();
  view.block<1, 3>(2, 0) = -f.transpose();
  view(0, 3) = -s.dot(eye);
  view(1, 3) = -u.dot(eye);
  view(2, 3) = f.dot(eye);
  return view;
}

}  // namespace

int main() {
  const clc::Vec3f eye{-3.f, 2.f, 5.f};
  const clc::Vec3f center{0.f, 0.f, 0.f};
  const clc::Vec3f up_hint{0.f, 1.f, 0.f};

  const clc::Mat4f clc_view = clc::lookAt(eye, center, up_hint);
  const Eigen::Matrix4f eigen_view =
      eigenViewLikeTrackball(Eigen::Vector3f(eye.x, eye.y, eye.z), Eigen::Vector3f(center.x, center.y, center.z),
                             Eigen::Vector3f(up_hint.x, up_hint.y, up_hint.z));

  if (!matricesNear(clc_view, eigen_view)) {
    std::fprintf(stderr, "linalg_eigen_parity_test: lookAt view matrix mismatch\n");
    return EXIT_FAILURE;
  }

  psiviz::TrackballCamera camera;
  camera.setViewport(596, 698, 2.f);
  camera.fitToBounds({-10.f, -5.f, -2.f}, {10.f, 5.f, 2.f});

  const clc::Mat4f cam_view = camera.viewMatrix();
  const clc::Mat4f clc_proj = camera.projectionMatrix();
  const clc::Mat4f clc_mvp = clc::multiply(clc_proj, cam_view);

  const clc::Vec3f focal = camera.focalPoint();
  const clc::Mat4f v = cam_view;
  const clc::Vec3f t{v(0, 3), v(1, 3), v(2, 3)};
  const clc::Vec3f cam_eye{-(v(0, 0) * t.x + v(1, 0) * t.y + v(2, 0) * t.z),
                           -(v(0, 1) * t.x + v(1, 1) * t.y + v(2, 1) * t.z),
                           -(v(0, 2) * t.x + v(1, 2) * t.y + v(2, 2) * t.z)};
  const clc::Vec3f cam_up{0.f, 1.f, 0.f};

  const Eigen::Matrix4f eigen_cam_view =
      eigenViewLikeTrackball(Eigen::Vector3f(cam_eye.x, cam_eye.y, cam_eye.z),
                             Eigen::Vector3f(focal.x, focal.y, focal.z), Eigen::Vector3f(cam_up.x, cam_up.y, cam_up.z));

  if (!matricesNear(cam_view, eigen_cam_view)) {
    std::fprintf(stderr, "linalg_eigen_parity_test: TrackballCamera view mismatch\n");
    return EXIT_FAILURE;
  }

  const Eigen::Matrix4f eigen_mvp = Eigen::Map<const Eigen::Matrix4f>(clc_proj.data()) * eigen_cam_view;
  if (!matricesNear(clc_mvp, eigen_mvp, 1e-3f)) {
    std::fprintf(stderr, "linalg_eigen_parity_test: mvp mismatch\n");
    return EXIT_FAILURE;
  }

  std::printf("linalg_eigen_parity_test: passed\n");
  return EXIT_SUCCESS;
}
