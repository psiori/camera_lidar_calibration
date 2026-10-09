import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root
    required property var cloudView
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
        RowLayout {
            Label { text: qsTr("Min") }
            TextField {
                implicitWidth: 56
                text: String(controller.viewerColormapMin)
                onEditingFinished: controller.viewerColormapMin = parseFloat(text)
            }
            Label { text: qsTr("Max") }
            TextField {
                implicitWidth: 56
                text: String(controller.viewerColormapMax)
                onEditingFinished: controller.viewerColormapMax = parseFloat(text)
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Label {
                text: qsTr("Blend")
                ToolTip.text: qsTr("Mix lidar scalar color with projected camera RGB (0 = lidar only, 1 = camera only).")
                ToolTip.delay: root.tooltipDelayMs
            }
            Slider {
                id: blendSlider
                from: 0
                to: 1
                value: cloudView.blendWeight
                onValueChanged: {
                    cloudView.blendWeight = value
                    controller.refreshViewerColors()
                }
                Layout.fillWidth: true
            }
        }
    }
}
