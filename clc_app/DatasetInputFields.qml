import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    id: root
    spacing: 10

    /// When true, Browse uses controller.browseImageFile / browsePcd* (active dataset).
    /// When false, Browse uses pickAddDataset* and updates local properties only.
    property bool useActiveDatasetBrowse: false
    property bool showIntrinsics: false
    property bool intrinsicsReadOnly: false

    property string imagePath: ""
    property string lidarPath: ""
    property string intrinsicsPath: ""

    signal imagePathEdited(string path)
    signal lidarPathEdited(string path)
    signal intrinsicsPathEdited(string path)

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
            text: root.imagePath
            placeholderText: qsTr("Image file (.png, .jpg, …)")
            onTextEdited: {
                root.imagePath = text
                root.imagePathEdited(text)
            }
        }
        Button {
            text: qsTr("Browse…")
            onClicked: {
                if (root.useActiveDatasetBrowse) {
                    controller.browseImageFile()
                } else {
                    const picked = controller.pickAddDatasetImage(root.imagePath)
                    if (picked.length > 0) {
                        root.imagePath = picked
                        root.imagePathEdited(picked)
                    }
                }
            }
        }
    }

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
            text: root.lidarPath
            placeholderText: qsTr("PCD directory or file")
            onTextEdited: {
                root.lidarPath = text
                root.lidarPathEdited(text)
            }
        }
        Button {
            text: qsTr("File…")
            onClicked: {
                if (root.useActiveDatasetBrowse) {
                    controller.browsePcdFile()
                } else {
                    const picked = controller.pickAddDatasetLidarFile(root.lidarPath)
                    if (picked.length > 0) {
                        root.lidarPath = picked
                        root.lidarPathEdited(picked)
                    }
                }
            }
        }
        Button {
            text: qsTr("Folder…")
            onClicked: {
                if (root.useActiveDatasetBrowse) {
                    controller.browsePcdFolder()
                } else {
                    const picked = controller.pickAddDatasetLidarFolder(root.lidarPath)
                    if (picked.length > 0) {
                        root.lidarPath = picked
                        root.lidarPathEdited(picked)
                    }
                }
            }
        }
    }

    Label {
        visible: root.showIntrinsics
        text: qsTr("Camera intrinsics (session)")
        font.bold: true
    }
    Label {
        visible: root.showIntrinsics
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        font.italic: true
        text: qsTr("OpenCV YAML (FileStorage); shared by all calibration datasets.")
    }
    RowLayout {
        visible: root.showIntrinsics
        Layout.fillWidth: true
        TextField {
            Layout.fillWidth: true
            readOnly: root.intrinsicsReadOnly
            text: root.intrinsicsPath
            placeholderText: qsTr("Intrinsics YAML (optional if already loaded)")
            onTextEdited: {
                if (!root.intrinsicsReadOnly) {
                    root.intrinsicsPath = text
                    root.intrinsicsPathEdited(text)
                }
            }
        }
        Button {
            visible: !root.intrinsicsReadOnly
            text: qsTr("Browse…")
            onClicked: {
                const picked = controller.pickAddDatasetIntrinsics(root.intrinsicsPath)
                if (picked.length > 0) {
                    root.intrinsicsPath = picked
                    root.intrinsicsPathEdited(picked)
                }
            }
        }
    }
}
