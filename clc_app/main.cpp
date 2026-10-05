#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "app_controller.hpp"

int main(int argc, char* argv[]) {
  QGuiApplication app(argc, argv);
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
