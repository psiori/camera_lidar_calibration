// Calibration camera image with correspondence markers on top of ZoomableImageView.
import QtQuick
import QtQuick.Controls

Item {
    id: root
    clip: true

    ZoomableImageView {
        id: viewport
        anchors.fill: parent
        source: controller.imagePath.length > 0 ? ("file://" + controller.imagePath) : ""
        navigationEnabled: controller.mainWorkTab === 1
        clickEnabled: controller.pickingAllowed && controller.mainWorkTab === 1
        showReticle: true
        onImageClicked: (u, v) => controller.pickImageCorrespondence(u, v)
    }

    Repeater {
        model: controller.correspondenceTable
        delegate: Item {
            required property var modelData
            readonly property real sx: viewport.imageDisplayWidth > 0 && viewport.nativeSourceSize.width > 0
                ? viewport.imageDisplayWidth / viewport.nativeSourceSize.width : 1
            readonly property real sy: viewport.imageDisplayHeight > 0 && viewport.nativeSourceSize.height > 0
                ? viewport.imageDisplayHeight / viewport.nativeSourceSize.height : 1
            readonly property real markerSize: modelData.selected ? 20 : (modelData.editable ? 16 : 14)
            x: viewport.imageX + modelData.u * sx - markerSize * 0.5
            y: viewport.imageY + modelData.v * sy - markerSize * 0.5
            width: markerSize
            height: markerSize
            z: modelData.selected ? 3 : 2

            Rectangle {
                anchors.fill: parent
                radius: width / 2
                color: modelData.pendingPick
                    ? "#26d9ff"
                    : (modelData.editable
                        ? (modelData.selected ? "#ffcc00" : "#ff6600")
                        : "#cccccc")
                border.color: modelData.pendingPick
                    ? "#00e5ff"
                    : (modelData.selected ? "#ffcc00" : (modelData.editable ? "#ff5500" : "#666666"))
                border.width: modelData.selected ? 3 : 2
            }
            Rectangle {
                width: 5
                height: 5
                radius: 2.5
                anchors.centerIn: parent
                color: "#ffffff"
            }
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 8
        height: pendingHint.implicitHeight + 12
        radius: 4
        color: "#cc1a3a4a"
        visible: controller.pendingCorrespondence3d || controller.pendingCorrespondence2d
        z: 10
        Label {
            id: pendingHint
            anchors.centerIn: parent
            width: parent.width - 16
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            color: "#00e5ff"
            text: controller.pendingCorrespondence3d && !controller.pendingCorrespondence2d
                ? "3D point picked — click the matching spot on the image"
                : (controller.pendingCorrespondence2d && !controller.pendingCorrespondence3d
                    ? "Image point picked — Shift+click the matching 3D point in the viewer"
                    : "Completing correspondence…")
        }
    }
}
