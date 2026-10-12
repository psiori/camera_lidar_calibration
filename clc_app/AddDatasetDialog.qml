import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Dialog {
    id: addDialog
    title: qsTr("Add calibration dataset")
    modal: true
    anchors.centerIn: parent
    width: Math.min(560, parent ? parent.width - 40 : 560)
    property string imagePath: ""
    property string lidarPath: ""
    property string intrinsicsPath: ""
    function resetFields() {
        imagePath = controller.addDatasetDefaultImagePath
        lidarPath = controller.addDatasetDefaultLidarPath
        intrinsicsPath = controller.addDatasetDefaultIntrinsicsPath
    }

    onAboutToShow: resetFields()

    footer: DialogButtonBox {
        standardButtons: DialogButtonBox.Ok | DialogButtonBox.Cancel
        onAccepted: {
            if (controller.addCalibrationDatasetFromPaths(addDialog.imagePath, addDialog.lidarPath,
                                                        addDialog.intrinsicsPath)) {
                addDialog.close()
            }
        }
        onRejected: addDialog.close()
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: qsTr("Choose inputs for a new dataset. Paths are fixed after you add the dataset.")
        }

        // Camera image
        Label {
            text: qsTr("Camera image")
            font.bold: true
        }
        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            font.italic: true
            text: qsTr("Still photo taken with the lidar camera for this capture.")
        }
        RowLayout {
            Layout.fillWidth: true
            TextField {
                Layout.fillWidth: true
                text: addDialog.imagePath
                placeholderText: qsTr("Image file (.png, .jpg, …)")
                onTextEdited: addDialog.imagePath = text
            }
            Button {
                text: qsTr("Browse…")
                onClicked: {
                    const picked = controller.pickAddDatasetImage(addDialog.imagePath)
                    if (picked.length > 0) addDialog.imagePath = picked
                }
            }
        }

        // LiDAR
        Label {
            text: qsTr("LiDAR scans")
            font.bold: true
        }
        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            font.italic: true
            text: qsTr("Folder of structured .pcd sweeps, or one .pcd file for a single scan.")
        }
        RowLayout {
            Layout.fillWidth: true
            TextField {
                Layout.fillWidth: true
                text: addDialog.lidarPath
                placeholderText: qsTr("PCD directory or file")
                onTextEdited: addDialog.lidarPath = text
            }
            Button {
                text: qsTr("Browse…")
                onClicked: {
                    const picked = controller.pickAddDatasetLidar(addDialog.lidarPath)
                    if (picked.length > 0) addDialog.lidarPath = picked
                }
            }
        }

        // Intrinsics
        Label {
            text: qsTr("Camera intrinsics (session)")
            font.bold: true
        }
        Label {
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            font.italic: true
            text: qsTr("OpenCV YAML (FileStorage); shared by all calibration datasets.")
        }
        RowLayout {
            Layout.fillWidth: true
            TextField {
                Layout.fillWidth: true
                text: addDialog.intrinsicsPath
                placeholderText: qsTr("Intrinsics YAML (optional if already loaded)")
                onTextEdited: addDialog.intrinsicsPath = text
            }
            Button {
                text: qsTr("Browse…")
                onClicked: {
                    const picked = controller.pickAddDatasetIntrinsics(addDialog.intrinsicsPath)
                    if (picked.length > 0) addDialog.intrinsicsPath = picked
                }
            }
        }
    }
}
