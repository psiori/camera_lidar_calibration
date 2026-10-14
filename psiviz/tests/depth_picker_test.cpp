#include <QtTest>

#include <clc/linalg.hpp>
#include "psiviz/depth_picker.hpp"

class DepthPickerTest : public QObject {
  Q_OBJECT

 private slots:
  void pickNearestOnScreen();
};

void DepthPickerTest::pickNearestOnScreen() {
  const float positions[] = {-1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f};
  const clc::Mat4f clip = clc::Mat4f::identity();
  const auto point =
      psiviz::DepthPicker::pickNearestOnScreen(positions, 3, 160, 120, 320, 240, 1.f, clip, 200.f);
  QVERIFY(point.has_value());
  QCOMPARE(point->x, 0.f);
  QCOMPARE(point->y, 0.f);
  QCOMPARE(point->z, 0.f);
}

QTEST_MAIN(DepthPickerTest)
#include "depth_picker_test.moc"
