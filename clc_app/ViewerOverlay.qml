// HUD over the 3D viewer: colormap controls, point size, and correspondence hints.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root
    required property var cloudView
    property bool keysBlocked: colormapSlider.dragging || blendSlider.pressed

    color: "#e8e8ec"
    border.color: "#b0b0b8"
    radius: 6
    opacity: 0.94
    implicitWidth: overlayLayout.implicitWidth + 16
    implicitHeight: overlayLayout.implicitHeight + 16

    readonly property int tooltipDelayMs: 1000

    ColumnLayout {
        id: overlayLayout
        anchors.centerIn: parent
        anchors.margins: 8
        spacing: 6

        Label {
            text: qsTr("Viewer")
            font.bold: true
        }
        RowLayout {
            Label { text: qsTr("Scalar") }
            ComboBox {
                model: [qsTr("Intensity"), qsTr("X"), qsTr("Y"), qsTr("Z")]
                currentIndex: controller.viewerScalarChannel
                onActivated: controller.viewerScalarChannel = currentIndex
            }
        }
        ColormapRangeSlider {
            id: colormapSlider
            Layout.fillWidth: true
            dataMin: controller.viewerDataMin
            dataMax: controller.viewerDataMax
            rangeMin: controller.viewerColormapMin
            rangeMax: controller.viewerColormapMax
            onRangeLive: (minValue, maxValue) => controller.previewViewerColormapRange(minValue, maxValue)
            onRangeCommitted: (minValue, maxValue) => controller.commitViewerColormapRange(minValue, maxValue)
        }
        RowLayout {
            Layout.fillWidth: true
            Label {
                text: qsTr("Blend")
                HoverHandler { id: blendTipHover }
                ToolTip.visible: blendTipHover.hovered
                ToolTip.text: qsTr("Mix lidar scalar color with projected camera RGB (0 = lidar only, 1 = camera only).")
                ToolTip.delay: root.tooltipDelayMs
            }
            Slider {
                id: blendSlider
                from: 0
                to: 1
                value: cloudView.blendWeight
                onValueChanged: cloudView.blendWeight = value
                Layout.fillWidth: true
            }
        }
    }
}
