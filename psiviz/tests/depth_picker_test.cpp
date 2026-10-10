#include <QtTest>

#include "psiviz/depth_picker.hpp"

class DepthPickerTest : public QObject {
  Q_OBJECT

 private slots:
  void decodePickIndexFromRgba();
  void pickNearestIndexOnScreen();
};

void DepthPickerTest::decodePickIndexFromRgba() {
  QVERIFY(!psiviz::DepthPicker::decodePickIndexFromRgba(0, 0, 0).has_value());
  const auto idx = psiviz::DepthPicker::decodePickIndexFromRgba(0, 0, 42);
  QVERIFY(idx.has_value());
  QCOMPARE(*idx, 41u);
}

void DepthPickerTest::pickNearestIndexOnScreen() {
  const float positions[] = {-1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f};
  const Eigen::Matrix4f clip = Eigen::Matrix4f::Identity();
  const auto index =
      psiviz::DepthPicker::pickNearestIndexOnScreen(positions, 3, 160, 120, 320, 240, 1.f, clip, 200.f);
  QVERIFY(index.has_value());
  QCOMPARE(*index, 1u);
}

QTEST_MAIN(DepthPickerTest)
#include "depth_picker_test.moc"
