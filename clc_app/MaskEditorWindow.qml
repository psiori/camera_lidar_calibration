import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

Window {
    id: maskWindow
    title: qsTr("Global mask editor")
    width: 960
    height: 720
    minimumWidth: 480
    minimumHeight: 360
    color: palette.window

    readonly property int tooltipDelayMs: 1000

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 8

        RowLayout {
            Layout.fillWidth: true
            visible: controller.calibrationDatasetIds.length > 0
            Label { text: qsTr("Preview dataset") }
            ComboBox {
                Layout.fillWidth: true
                model: controller.calibrationDatasetIds
                currentIndex: controller.activeCalibrationDatasetIndex
                onActivated: controller.setMaskPreviewDatasetIndex(currentIndex)
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Label { text: qsTr("Channel") }
            ComboBox {
                model: [qsTr("Range"), qsTr("Intensity")]
                currentIndex: controller.rangeChannel
                onActivated: controller.rangeChannel = currentIndex
            }
            Label {
                text: qsTr("Y stretch (×)")
                ToolTip.text: qsTr("Vertical scale multiplier on the range image (1 = native aspect).")
                ToolTip.delay: maskWindow.tooltipDelayMs
            }
            SpinBox {
                from: 1
                to: 8
                stepSize: 1
                value: Math.max(1, Math.min(8, Math.round(controller.rangeYStretch)))
                onValueModified: controller.rangeYStretch = value
            }
            Item { Layout.fillWidth: true }
        }

        ColormapRangeSlider {
            Layout.fillWidth: true
            dataMin: controller.rangeDataMin
            dataMax: controller.rangeDataMax
            rangeMin: controller.rangeColormapMin
            rangeMax: controller.rangeColormapMax
            onRangeLive: (minValue, maxValue) => controller.previewRangeColormapRange(minValue, maxValue)
            onRangeCommitted: (minValue, maxValue) => controller.commitRangeColormapRange(minValue, maxValue)
        }

        RowLayout {
            Layout.fillWidth: true
            Button {
                text: qsTr("Close polygon")
                enabled: rangeView.drawing
                onClicked: rangeView.closePolygon()
            }
            Button {
                text: qsTr("Undo vertex")
                enabled: rangeView.drawing
                onClicked: rangeView.undoVertex()
            }
            Button {
                text: qsTr("Cancel draw")
                enabled: rangeView.drawing
                onClicked: rangeView.cancelPolygon()
            }
            Button {
                text: qsTr("Keep")
                enabled: controller.maskPolygonPending
                onClicked: controller.applyPendingMaskKeep()
                ToolTip.text: qsTr("Keep points inside the closed polygon (restore or include region).")
                ToolTip.delay: maskWindow.tooltipDelayMs
            }
            Button {
                text: qsTr("Remove")
                enabled: controller.maskPolygonPending
                onClicked: controller.applyPendingMaskRemove()
                ToolTip.text: qsTr("Remove points inside the closed polygon (exclude region).")
                ToolTip.delay: maskWindow.tooltipDelayMs
            }
            Button {
                text: qsTr("Discard polygon")
                enabled: controller.maskPolygonPending
                onClicked: controller.discardPendingMaskPolygon()
            }
        }

        RangeImageItem {
            id: rangeView
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 120
            fillBounds: true
            image: controller.rangeImage
            maskData: controller.maskPreview
            maskWidth: controller.maskWidth
            maskHeight: controller.maskHeight
            maskController: controller
            closedPolygonOverlay: controller.pendingMaskPolygonVertices
        }

        Connections {
            target: controller
            function onMaskPolygonPendingChanged() {
                if (!controller.maskPolygonPending)
                    rangeView.clearPendingPolygon()
            }
        }

        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: controller.maskPolygonPending
                ? qsTr("Polygon ready — click Keep or Remove to update the mask, then draw another if needed.")
                : (controller.maskPath.length > 0 ? (qsTr("Mask file: ") + controller.maskPath)
                    : qsTr("Click to add vertices (edges snap to the border). Finish with the green first vertex, double-click, or Close polygon."))
        }
    }
}
