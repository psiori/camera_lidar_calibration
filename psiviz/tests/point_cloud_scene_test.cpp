#include <cstdlib>

#include <QtQml/qqmlextensionplugin.h>
#include <QtTest>

#include <QImage>
#include <QQmlComponent>
#include <QQuickItem>
#include <QQuickView>
#include <QVector>

#include "psiviz/point_cloud_item.hpp"

Q_IMPORT_QML_PLUGIN(PsivizPlugin)

namespace {

bool isClearColor(QRgb pixel) {
  const int r = qRed(pixel);
  const int g = qGreen(pixel);
  const int b = qBlue(pixel);
  return std::abs(r - 20) <= 8 && std::abs(g - 23) <= 8 && std::abs(b - 28) <= 8;
}

int countNonClearPixels(const QImage& frame) {
  const QImage image = frame.convertToFormat(QImage::Format_ARGB32);
  int count = 0;
  for (int y = 0; y < image.height(); ++y) {
    const auto* row = reinterpret_cast<const QRgb*>(image.constScanLine(y));
    for (int x = 0; x < image.width(); ++x) {
      if (!isClearColor(row[x])) {
        ++count;
      }
    }
  }
  return count;
}

}  // namespace

class PointCloudSceneTest : public QObject {
  Q_OBJECT

 private slots:
  void syntheticPointsAreVisible();
};

void PointCloudSceneTest::syntheticPointsAreVisible() {
  QQuickView window;
  window.setColor(Qt::white);
  window.setResizeMode(QQuickView::SizeRootObjectToView);
  window.resize(320, 240);

  QQmlComponent component(window.engine());
  component.setData(
      "import QtQuick\n"
      "import Psiviz\n"
      "PointCloudScene {\n"
      "  objectName: \"cloud\"\n"
      "  pointSize: 18\n"
      "}\n",
      QUrl());
  QVERIFY2(component.isReady(), qPrintable(component.errorString()));

  auto* root = qobject_cast<QQuickItem*>(component.create());
  QVERIFY(root != nullptr);
  window.setContent(QUrl(), &component, root);
  window.show();
  QVERIFY(QTest::qWaitForWindowExposed(&window));

  auto* cloud = qobject_cast<psiviz::PointCloudItem*>(window.rootObject());
  QVERIFY(cloud != nullptr);

  QVector<float> positions;
  QVector<float> scalars;
  QVector<float> rgb;
  for (int iz = 0; iz <= 8; ++iz) {
    for (int iy = 0; iy <= 8; ++iy) {
      for (int ix = 0; ix <= 8; ++ix) {
        positions << (ix / 4.f - 1.f) << (iy / 4.f - 1.f) << (iz / 4.f - 1.f);
        scalars << (ix / 8.f);
        rgb << 1.f << 0.f << 0.f;
      }
    }
  }
  cloud->setPointCloudData(positions, scalars, rgb);
  cloud->frameCameraToContent();

  QImage frame;
  int colored = 0;
  for (int attempt = 0; attempt < 20; ++attempt) {
    QTest::qWait(50);
    frame = window.grabWindow();
    colored = frame.isNull() ? 0 : countNonClearPixels(frame);
    if (colored > 30) {
      break;
    }
  }

  const QColor center = frame.isNull() ? QColor() : frame.pixelColor(frame.width() / 2, frame.height() / 2);
  qInfo() << "view geom" << cloud->width() << cloud->height() << "frame" << frame.size() << "center" << center
          << "nonclear" << colored;
  QVERIFY(!frame.isNull());
  QVERIFY2(colored > 30, "expected colored points; framebuffer stayed at the clear color");

  window.close();
}

QTEST_MAIN(PointCloudSceneTest)
#include "point_cloud_scene_test.moc"
