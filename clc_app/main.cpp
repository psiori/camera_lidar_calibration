#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "app_controller.hpp"

/// Desktop entry point: creates AppController, exposes it to QML as `controller`, loads Main.qml.
int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  AppController controller;
  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty("controller", &controller);
  engine.rootContext()->setContextProperty("appVersion", QStringLiteral("0.1.0"));
  engine.loadFromModule("CameraLidarCalibration", "Main");
  if (engine.rootObjects().isEmpty()) {
    return 1;
  }
  return app.exec();
}
