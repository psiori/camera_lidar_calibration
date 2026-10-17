#include <QtTest>

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQuickItem>
#include <QQuickWindow>
#include <QSignalSpy>

#include "app_controller.hpp"
#include "range_image_item.hpp"

namespace {

constexpr int kNativeWidth = 96;
constexpr int kNativeHeight = 24;
constexpr double kYStretch = 2.0;

RangeImageItem* findMaskRangeView(QObject* root) {
  if (!root) {
    return nullptr;
  }
  auto* item = root->findChild<RangeImageItem*>(QStringLiteral("maskRangeView"), Qt::FindChildrenRecursively);
  return item;
}

void polishAndWait(QQuickWindow* window, int ms = 80) {
  QVERIFY(window);
  window->show();
  QVERIFY(QTest::qWaitForWindowExposed(window));
  QCoreApplication::processEvents(QEventLoop::AllEvents, ms);
  QTest::qWait(ms);
}

}  // namespace

class MaskEditorPolygonTest : public QObject {
  Q_OBJECT

 private slots:
  void closedPolygonStaysVisibleAfterCloseWithMarginVertices();
  void closedPolygonVisibleWhenMaskReferenceLargerThanRangeImage();
  void openPolygonVerticesPersistAcrossColormapSliderPreview();
};

void MaskEditorPolygonTest::closedPolygonStaysVisibleAfterCloseWithMarginVertices() {
  AppController controller;
  controller.installSyntheticMaskEditorScan(kNativeWidth, kNativeHeight, kYStretch);
  QVERIFY(!controller.rangeImage().isNull());

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty(QStringLiteral("controller"), &controller);

  QQmlComponent component(&engine);
  component.loadFromModule(QStringLiteral("CameraLidarCalibrationTest"), QStringLiteral("MaskEditorWindow"));
  if (component.isError()) {
    for (const QQmlError& err : component.errors()) {
      qWarning() << err.toString();
    }
  }
  QVERIFY2(!component.isError(), "MaskEditorWindow must load for this test");
  QObject* root = component.create();
  QVERIFY(root);

  auto* window = qobject_cast<QQuickWindow*>(root);
  QVERIFY2(window, "MaskEditorWindow must be a QQuickWindow");
  polishAndWait(window);

  RangeImageItem* range_view = findMaskRangeView(window);
  QVERIFY2(range_view, "maskRangeView RangeImageItem not found in MaskEditorWindow");

  range_view->setWidth(640);
  range_view->setHeight(360);
  polishAndWait(window);

  const QRectF dest = range_view->imageDestinationRectForTest();
  const QRectF margin = range_view->interactionMarginRectForTest();
  QVERIFY(dest.isValid());
  QVERIFY(margin.contains(dest));

  // One vertex on the image, two from clicks outside the rendered image (in the margin).
  const QPointF inside(dest.center());
  const QPointF outside_left(margin.left() + 4.0, dest.center().y());
  const QPointF outside_bottom(dest.center().x(), margin.bottom() - 4.0);
  QVERIFY(!dest.contains(outside_left));
  QVERIFY(!dest.contains(outside_bottom));
  QVERIFY(margin.contains(outside_left));
  QVERIFY(margin.contains(outside_bottom));

  range_view->clickAtDisplayPosition(inside.x(), inside.y());
  range_view->clickAtDisplayPosition(outside_left.x(), outside_left.y());
  range_view->clickAtDisplayPosition(outside_bottom.x(), outside_bottom.y());
  QVERIFY(range_view->drawing());

  range_view->closePolygon();
  QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
  QTest::qWait(50);

  // Mask preview and colormap refresh must not clear the closed polygon overlay.
  controller.previewRangeColormapRange(0.0, 40.0);
  controller.commitRangeColormapRange(0.0, 40.0);
  QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
  QTest::qWait(50);

  QVERIFY2(controller.maskPolygonPending(),
             "Closing the polygon must leave a pending mask op until Keep/Remove/Discard");
  QCOMPARE_GE(range_view->closedPolygonVertexCount(), 3);

  QVERIFY2(range_view->containsPolygonOutlinePixels(),
             "Closed polygon outline must remain visible in the mask editor after close "
             "(regression: invisible polygon while maskPolygonPending stays true)");

  // Still awaiting decision: further clicks must not start a new polygon.
  const int vertices_before = range_view->closedPolygonVertexCount();
  range_view->clickAtDisplayPosition(inside.x(), inside.y());
  QCOMPARE(range_view->closedPolygonVertexCount(), vertices_before);
  QVERIFY(!range_view->drawing());

  window->close();
  delete root;
}

void MaskEditorPolygonTest::closedPolygonVisibleWhenMaskReferenceLargerThanRangeImage() {
  AppController controller;
  controller.installSyntheticMaskEditorScan(kNativeWidth, kNativeHeight, kYStretch);
  controller.setReferenceScanSizeForTest(kNativeWidth * 2, kNativeHeight * 2);
  QCOMPARE(controller.maskWidth(), kNativeWidth * 2);
  QCOMPARE(controller.rangeImageNativeWidth(), kNativeWidth);

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty(QStringLiteral("controller"), &controller);

  QQmlComponent component(&engine);
  component.loadFromModule(QStringLiteral("CameraLidarCalibrationTest"), QStringLiteral("MaskEditorWindow"));
  QVERIFY2(!component.isError(), "MaskEditorWindow must load for this test");
  QObject* root = component.create();
  QVERIFY(root);

  auto* window = qobject_cast<QQuickWindow*>(root);
  polishAndWait(window);

  RangeImageItem* range_view = findMaskRangeView(window);
  QVERIFY(range_view);
  range_view->setWidth(640);
  range_view->setHeight(360);
  polishAndWait(window);

  const QRectF dest = range_view->imageDestinationRectForTest();
  const QPointF a(dest.left() + dest.width() * 0.25, dest.center().y());
  const QPointF b(dest.right() - dest.width() * 0.25, dest.top() + dest.height() * 0.2);
  const QPointF c(dest.center().x(), dest.bottom() - dest.height() * 0.15);

  range_view->clickAtDisplayPosition(a.x(), a.y());
  range_view->clickAtDisplayPosition(b.x(), b.y());
  range_view->clickAtDisplayPosition(c.x(), c.y());
  range_view->closePolygon();
  QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

  QVERIFY(controller.maskPolygonPending());
  QVERIFY2(range_view->containsPolygonOutlinePixels(),
           "Outline must stay on-screen when mask reference raster is larger than the preview range image");

  window->close();
  delete root;
}

void MaskEditorPolygonTest::openPolygonVerticesPersistAcrossColormapSliderPreview() {
  AppController controller;
  controller.installSyntheticMaskEditorScan(kNativeWidth, kNativeHeight, kYStretch);
  QVERIFY(!controller.rangeImage().isNull());

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty(QStringLiteral("controller"), &controller);

  QQmlComponent component(&engine);
  component.loadFromModule(QStringLiteral("CameraLidarCalibrationTest"), QStringLiteral("MaskEditorWindow"));
  QVERIFY2(!component.isError(), "MaskEditorWindow must load for this test");
  QObject* root = component.create();
  QVERIFY(root);

  auto* window = qobject_cast<QQuickWindow*>(root);
  polishAndWait(window);

  RangeImageItem* range_view = findMaskRangeView(window);
  QVERIFY(range_view);
  range_view->setWidth(640);
  range_view->setHeight(360);
  polishAndWait(window);

  const QRectF dest = range_view->imageDestinationRectForTest();
  const QPointF a(dest.left() + dest.width() * 0.3, dest.center().y());
  const QPointF b(dest.center().x(), dest.top() + dest.height() * 0.25);
  range_view->clickAtDisplayPosition(a.x(), a.y());
  range_view->clickAtDisplayPosition(b.x(), b.y());
  QVERIFY(range_view->drawing());
  QCOMPARE(range_view->closedPolygonVertexCount(), 0);

  controller.previewRangeColormapRange(0.0, 40.0);
  controller.commitRangeColormapRange(0.0, 40.0);
  QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

  QVERIFY2(range_view->drawing(),
           "In-progress polygon must survive colormap slider preview (regression: setImage reset)");
  QVERIFY2(range_view->containsPolygonOutlinePixels(),
           "Open polygon edges must remain visible after colormap refresh");

  window->close();
  delete root;
}

int main(int argc, char* argv[]) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  MaskEditorPolygonTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "mask_editor_polygon_test.moc"
