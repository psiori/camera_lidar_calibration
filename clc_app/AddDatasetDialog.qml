// Modal dialog to pick image, lidar folder/file, and intrinsics for a new calibration dataset.
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

        DatasetInputFields {
            Layout.fillWidth: true
            showIntrinsics: true
            imagePath: addDialog.imagePath
            lidarPath: addDialog.lidarPath
            intrinsicsPath: addDialog.intrinsicsPath
            onImagePathEdited: path => { addDialog.imagePath = path }
            onLidarPathEdited: path => { addDialog.lidarPath = path }
            onIntrinsicsPathEdited: path => { addDialog.intrinsicsPath = path }
        }
    }
}
