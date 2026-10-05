import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: root
    width: 1280
    height: 800
    visible: true
    title: "Camera-LiDAR Calibration"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        Label {
            text: "Camera-LiDAR calibration"
            font.bold: true
            font.pixelSize: 18
        }

        Label {
            text: controller.statusMessage
            Layout.fillWidth: true
        }

        Label {
            text: "Inspection note: image and scans are assumed to depict the same stationary scene even when not time-synchronized."
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        TabBar {
            id: tabs
            Layout.fillWidth: true
            TabButton { text: "Calibration" }
            TabButton { text: "Inspection" }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabs.currentIndex

            ColumnLayout {
                spacing: 8
                GroupBox {
                    title: "Scan time"
                    Layout.fillWidth: true
                    ColumnLayout {
                        ComboBox {
                            id: timeUnit
                            model: ["Seconds", "Nanoseconds"]
                            currentIndex: controller.timeUnit
                            onActivated: controller.timeUnit = currentIndex
                        }
                        ComboBox {
                            id: timeOrigin
                            model: ["FirstPoint", "ScanStart"]
                            currentIndex: controller.timeOrigin
                            onActivated: controller.timeOrigin = currentIndex
                        }
                        RowLayout {
                            Label { text: "Scan duration (s)" }
                            TextField {
                                id: scanDuration
                                text: controller.scanDuration > 0 ? controller.scanDuration : ""
                                placeholderText: "required for ScanStart"
                                onEditingFinished: controller.scanDuration = parseFloat(text)
                            }
                        }
                    }
                }
                RowLayout {
                    Button {
                        text: "Run fusion"
                        onClicked: controller.runFusion()
                    }
                    Button {
                        text: "Run alignment"
                        onClicked: controller.runAlignment()
                    }
                    Label { text: "Points: " + controller.pointCount }
                }
            }

            ColumnLayout {
                spacing: 8
                TextField {
                    id: imagePath
                    placeholderText: "Image path"
                    Layout.fillWidth: true
                    onEditingFinished: controller.setImagePath(text)
                }
                TextField {
                    id: pcdDir
                    placeholderText: "PCD directory"
                    Layout.fillWidth: true
                    onEditingFinished: controller.setPcdDirectory(text)
                }
                Button {
                    text: "Fuse inspection scans"
                    onClicked: controller.runFusion()
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            TextField {
                id: sessionDir
                placeholderText: "Session directory"
                Layout.fillWidth: true
            }
            Button {
                text: "Load"
                onClicked: controller.loadSession(sessionDir.text)
            }
            Button {
                text: "Save"
                onClicked: controller.saveSession(sessionDir.text)
            }
        }
    }
}
