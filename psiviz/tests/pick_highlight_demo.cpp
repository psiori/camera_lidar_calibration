// Demonstrates Shift+click (or --auto) point picking and red highlight on a synthetic cloud.
// Build: psiviz_pick_highlight_demo
// Run:   psiviz_pick_highlight_demo           # interactive
//        psiviz_pick_highlight_demo --auto   # self-test, exit 0 on success

#include <cmath>
#include <cstdlib>

#include <QCommandLineParser>
#include <QEventLoop>
#include <QGuiApplication>
#include <QQmlComponent>
#include <QQuickView>
#include <QTimer>
#include <QtTest>
#include <QtQml/qqmlextensionplugin.h>

#include "psiviz/point_cloud_item.hpp"

Q_IMPORT_QML_PLUGIN(PsivizPlugin)

namespace {

QVector<float> makeSyntheticGrid(int grid_n, QVector<float>& scalars, QVector<float>& rgb) {
  QVector<float> positions;
  scalars.clear();
  rgb.clear();
  const float step = 2.f / static_cast<float>(grid_n - 1);
  for (int iz = 0; iz < grid_n; ++iz) {
    for (int iy = 0; iy < grid_n; ++iy) {
      for (int ix = 0; ix < grid_n; ++ix) {
        const float x = -1.f + static_cast<float>(ix) * step;
        const float y = -1.f + static_cast<float>(iy) * step;
        const float z = -1.f + static_cast<float>(iz) * step;
        positions << x << y << z;
        scalars << 0.2f;
        rgb << 0.2f << 0.55f << 0.9f;
      }
    }
  }
  return positions;
}

}  // namespace

int main(int argc, char* argv[]) {
  QGuiApplication app(argc, argv);
  QCoreApplication::setApplicationName(QStringLiteral("psiviz_pick_highlight_demo"));

  QCommandLineParser parser;
  parser.setApplicationDescription(
      QStringLiteral("Synthetic point cloud: Shift+left-click picks a point (bright red + sphere marker)."));
  parser.addHelpOption();
  const QCommandLineOption auto_opt(QStringLiteral("auto"),
                                    QStringLiteral("Run automated pick/highlight check and exit."));
  parser.addOption(auto_opt);
  parser.process(app);

  QQuickView window;
  window.setColor(QColor(20, 23, 28));
  window.setResizeMode(QQuickView::SizeRootObjectToView);
  window.resize(480, 360);

  QQmlComponent component(window.engine());
  component.setData(
      "import QtQuick\n"
      "import Psiviz\n"
      "PointCloudScene {\n"
      "  objectName: \"cloud\"\n"
      "  anchors.fill: parent\n"
      "  pointSize: 22\n"
      "}\n",
      QUrl());
  if (!component.isReady()) {
    qCritical() << component.errorString();
    return 1;
  }

  auto* root = qobject_cast<QQuickItem*>(component.create());
  if (!root) {
    qCritical() << "failed to create PointCloudScene";
    return 1;
  }
  window.setContent(QUrl(), &component, root);
  window.show();

  if (!QTest::qWaitForWindowExposed(&window)) {
    qCritical() << "window failed to expose";
    return 1;
  }

  auto* cloud = qobject_cast<psiviz::PointCloudItem*>(window.rootObject());
  if (!cloud) {
    qCritical() << "root is not PointCloudItem";
    return 1;
  }

  QVector<float> scalars;
  QVector<float> rgb;
  const QVector<float> positions = makeSyntheticGrid(5, scalars, rgb);
  cloud->setBlendWeight(1.0);
  cloud->setScalarMin(0.0);
  cloud->setScalarMax(1.0);
  cloud->setPointCloudData(positions, scalars, rgb);
  cloud->frameCameraToContent();
  cloud->setMarkerSphereRadius(0.05);

  const bool auto_mode = parser.isSet(auto_opt);

  int pick_count = 0;
  QObject::connect(cloud, &psiviz::PointCloudItem::pointPicked, &app, [&](double x, double y, double z) {
    qInfo().noquote() << QStringLiteral("Picked xyz=(%1,%2,%3) — sphere marker")
                             .arg(x, 0, 'f', 4)
                             .arg(y, 0, 'f', 4)
                             .arg(z, 0, 'f', 4);
    QVector<double> marker;
    marker << x << y << z;
    cloud->setSphereMarkerPositions(marker);
    ++pick_count;
  });

  if (auto_mode) {
    QEventLoop loop;
    QTimer::singleShot(400, cloud, [cloud]() {
      const int cx = std::max(1, static_cast<int>(cloud->width())) / 2;
      const int cy = std::max(1, static_cast<int>(cloud->height())) / 2;
      qInfo().noquote() << QStringLiteral("Auto pick at viewport center (%1, %2)").arg(cx).arg(cy);
      cloud->requestPickAt(cx, cy);
    });

    QTimer::singleShot(2500, &loop, &QEventLoop::quit);
    loop.exec();

    if (pick_count == 0) {
      qCritical() << "AUTO FAIL: no pointPicked signal received (pick pipeline or update scheduling broken)";
      return 2;
    }
    qInfo().noquote() << QStringLiteral("AUTO PASS: pointPicked received (sphere marker placed)");
    return 0;
  }

  qInfo().noquote() << QStringLiteral("Interactive mode: Shift+left-click to pick (orange sphere marker).");
  qInfo().noquote() << QStringLiteral("Close the window to exit.");
  return app.exec();
}
