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

bool isMostlyRed(const QColor& c) {
  return c.red() > 200 && c.green() < 80 && c.blue() < 80;
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

  QVector<qint64> picked_indices;
  QObject::connect(cloud, &psiviz::PointCloudItem::pointPicked, &app, [&](qint64 index) {
    qInfo().noquote() << QStringLiteral("Picked point index %1 — highlighting red%2")
                             .arg(index)
                             .arg(auto_mode ? QString() : QStringLiteral(" + sphere"));
    cloud->highlightPointIndex(index);
    if (!auto_mode) {
      const int i = static_cast<int>(index);
      if (i >= 0 && i * 3 + 2 < positions.size()) {
        QVector<double> marker;
        marker << positions[i * 3 + 0] << positions[i * 3 + 1] << positions[i * 3 + 2];
        cloud->setSphereMarkerPositions(marker);
      }
    }
    picked_indices.push_back(index);
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

    if (picked_indices.isEmpty()) {
      qCritical() << "AUTO FAIL: no pointPicked signal received (pick pipeline or update scheduling broken)";
      return 2;
    }
    qInfo() << "AUTO: picked index" << picked_indices.back();

    QImage after;
    for (int attempt = 0; attempt < 25; ++attempt) {
      QTest::qWait(40);
      after = window.grabWindow();
      const QColor center = after.pixelColor(after.width() / 2, after.height() / 2);
      if (isMostlyRed(center)) {
        qInfo().noquote() << QStringLiteral("AUTO PASS: center pixel %1,%2,%3 after highlight")
                                 .arg(center.red())
                                 .arg(center.green())
                                 .arg(center.blue());
        return 0;
      }
    }
    const QColor center = after.pixelColor(after.width() / 2, after.height() / 2);
    qCritical().noquote() << QStringLiteral("AUTO FAIL: center not red after pick (got %1,%2,%3)")
                                   .arg(center.red())
                                   .arg(center.green())
                                   .arg(center.blue());
    return 3;
  }

  qInfo().noquote() << QStringLiteral("Interactive mode: Shift+left-click points in the view to pick/highlight.");
  qInfo().noquote() << QStringLiteral("Close the window to exit.");
  return app.exec();
}
