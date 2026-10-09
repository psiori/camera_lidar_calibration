#include <cmath>

#include <QtTest>

#include "psiviz/trackball_camera.hpp"

namespace {

Eigen::Vector3f eyeFromView(const Eigen::Matrix4f& view) {
  const Eigen::Matrix3f r = view.block<3, 3>(0, 0);
  const Eigen::Vector3f t(view(0, 3), view(1, 3), view(2, 3));
  return -r.transpose() * t;
}

}  // namespace

class TrackballCameraTest : public QObject {
  Q_OBJECT

 private slots:
  void rotatePreservesFocalAndDistance();
  void rotateDoesNotClampAfterManyVerticalDrags();
};

void TrackballCameraTest::rotatePreservesFocalAndDistance() {
  psiviz::TrackballCamera camera;
  camera.setViewport(800, 600, 1.f);
  camera.fitToBounds(Eigen::Vector3f(-1.f, -1.f, -1.f), Eigen::Vector3f(1.f, 1.f, 1.f));

  const Eigen::Vector3f focal = camera.focalPoint();
  const float d0 = (eyeFromView(camera.viewMatrix()) - focal).norm();

  camera.rotate(40.f, 25.f);
  camera.rotate(-15.f, 60.f);

  QVERIFY((camera.focalPoint() - focal).norm() < 1e-4f);
  const float d1 = (eyeFromView(camera.viewMatrix()) - focal).norm();
  QVERIFY(std::abs(d0 - d1) < 1e-3f);
}

void TrackballCameraTest::rotateDoesNotClampAfterManyVerticalDrags() {
  psiviz::TrackballCamera camera;
  camera.setViewport(1000, 800, 1.f);
  camera.fitToBounds(Eigen::Vector3f(-2.f, -2.f, -2.f), Eigen::Vector3f(2.f, 2.f, 2.f));

  const Eigen::Matrix4f prev = camera.viewMatrix();
  for (int i = 0; i < 80; ++i) {
    camera.rotate(0.f, 12.f);
  }
  const float delta = (camera.viewMatrix() - prev).norm();
  QVERIFY2(delta > 0.05f, "view should keep changing; pitch clamp would freeze orientation");
}

QTEST_MAIN(TrackballCameraTest)
#include "trackball_camera_test.moc"
