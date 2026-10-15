// Per-dataset camera image mask for NID optimization (polygon keep/remove).
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

Window {
    id: imageMaskWindow
    title: qsTr("Image optimization mask editor")
    width: 960
    height: 720
    minimumWidth: 480
    minimumHeight: 360
    color: palette.window
    modality: Qt.ApplicationModal

    readonly property int tooltipDelayMs: 1000

    function resetPendingPolygonIfNeeded() {
        if (controller.imageMaskPolygonPending)
            controller.discardPendingImageMaskPolygon()
        imageView.clearPendingPolygon()
    }

    onVisibleChanged: {
        controller.setImageMaskEditorOpen(visible)
        if (!visible)
            resetPendingPolygonIfNeeded()
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 8

        RowLayout {
            Layout.fillWidth: true
            Button {
                text: qsTr("Close polygon")
                enabled: imageView.drawing
                onClicked: imageView.closePolygon()
            }
            Button {
                text: qsTr("Undo vertex")
                enabled: imageView.drawing
                onClicked: imageView.undoVertex()
            }
            Button {
                text: qsTr("Cancel draw")
                enabled: imageView.drawing
                onClicked: imageView.cancelPolygon()
            }
            Button {
                text: qsTr("Keep")
                enabled: controller.imageMaskPolygonPending
                onClicked: {
                    controller.applyPendingImageMaskKeep()
                    imageView.clearPendingPolygon()
                }
                ToolTip.text: qsTr("Keep pixels inside the closed polygon for NID cost.")
                ToolTip.delay: imageMaskWindow.tooltipDelayMs
            }
            Button {
                text: qsTr("Remove")
                enabled: controller.imageMaskPolygonPending
                onClicked: {
                    controller.applyPendingImageMaskRemove()
                    imageView.clearPendingPolygon()
                }
                ToolTip.text: qsTr("Exclude pixels inside the closed polygon from NID cost.")
                ToolTip.delay: imageMaskWindow.tooltipDelayMs
            }
            Button {
                text: qsTr("Discard polygon")
                enabled: controller.imageMaskPolygonPending
                onClicked: {
                    controller.discardPendingImageMaskPolygon()
                    imageView.clearPendingPolygon()
                }
            }
        }

        RangeImageItem {
            id: imageView
            objectName: "imageMaskView"
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 120
            fillBounds: true
            imageOptimizationMaskMode: true
            image: controller.cameraPreviewImage
            maskData: controller.imageMaskPreview
            maskWidth: controller.imageMaskWidth
            maskHeight: controller.imageMaskHeight
            rangeNativeWidth: controller.imageMaskWidth
            rangeNativeHeight: controller.imageMaskHeight
            maskController: controller
        }

        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: controller.imageMaskPolygonPending
                ? qsTr("Polygon ready — click Keep or Remove to update the image mask.")
                : (controller.imageMaskPath.length > 0
                    ? (qsTr("Mask file: ") + controller.imageMaskPath)
                    : qsTr("Click to add vertices. Finish with the green first vertex, double-click, or Close polygon."))
        }
    }
}
