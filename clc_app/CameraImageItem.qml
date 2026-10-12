import QtQuick
import QtQuick.Controls

Item {
    id: root
    clip: true

    // 1.0 = letterboxed fit; never shrink below fit-to-view.
    property real minZoom: 1.0
    property real maxZoom: 12.0
    property color reticleColor: "#00e5ff"
    property real reticleOuterSize: 28
    property real reticleLineExtension: 10

    property real fitScale: 1.0
    property real zoom: 1.0
    property real panX: 0
    property real panY: 0

    property real panDragStartX: 0
    property real panDragStartY: 0

    readonly property real displayScale: fitScale * zoom

    function updateFitScale() {
        if (photo.status !== Image.Ready || photo.sourceSize.width <= 0 || photo.sourceSize.height <= 0)
            return
        if (root.width <= 0 || root.height <= 0)
            return
        const newFit = Math.min(root.width / photo.sourceSize.width,
                                root.height / photo.sourceSize.height)
        if (Math.abs(newFit - fitScale) < 1e-9)
            return

        const cx = root.width * 0.5
        const cy = root.height * 0.5
        const oldFit = fitScale
        const oldW = photo.sourceSize.width * oldFit * zoom
        const oldH = photo.sourceSize.height * oldFit * zoom
        const oldCenterX = (root.width - oldW) * 0.5
        const oldCenterY = (root.height - oldH) * 0.5
        const ix = oldW > 0 ? (cx - oldCenterX - panX) / oldW : 0.5
        const iy = oldH > 0 ? (cy - oldCenterY - panY) / oldH : 0.5

        fitScale = newFit

        const newW = photo.sourceSize.width * fitScale * zoom
        const newH = photo.sourceSize.height * fitScale * zoom
        const newCenterX = (root.width - newW) * 0.5
        const newCenterY = (root.height - newH) * 0.5
        panX = cx - newCenterX - ix * newW
        panY = cy - newCenterY - iy * newH
        clampPan()
    }

    function resetView() {
        updateFitScale()
        zoom = 1.0
        panX = 0
        panY = 0
        clampPan()
    }

    function clampPan() {
        if (photo.width <= 0 || photo.height <= 0)
            return
        const margin = 32
        const centerX = (root.width - photo.width) * 0.5
        const centerY = (root.height - photo.height) * 0.5
        const minLeft = margin - photo.width
        const maxLeft = root.width - margin
        const minTop = margin - photo.height
        const maxTop = root.height - margin
        if (photo.width <= root.width - 2 * margin)
            panX = 0
        else
            panX = Math.min(maxLeft - centerX, Math.max(minLeft - centerX, panX))
        if (photo.height <= root.height - 2 * margin)
            panY = 0
        else
            panY = Math.min(maxTop - centerY, Math.max(minTop - centerY, panY))
    }

    function imagePixelFromView(viewX, viewY) {
        if (photo.sourceSize.width <= 0 || photo.width <= 0 || photo.height <= 0)
            return null
        const sx = photo.width / photo.sourceSize.width
        const sy = photo.height / photo.sourceSize.height
        const localX = viewX - photo.x
        const localY = viewY - photo.y
        const u = localX / sx
        const v = localY / sy
        if (u < 0 || v < 0 || u > photo.sourceSize.width || v > photo.sourceSize.height)
            return null
        return { u: u, v: v }
    }

    function zoomAt(factor, cx, cy) {
        const oldZoom = zoom
        const newZoom = Math.min(maxZoom, Math.max(minZoom, zoom * factor))
        if (Math.abs(newZoom - oldZoom) < 1e-6)
            return

        const oldW = photo.sourceSize.width * fitScale * oldZoom
        const oldH = photo.sourceSize.height * fitScale * oldZoom
        const centerX = (root.width - oldW) * 0.5
        const centerY = (root.height - oldH) * 0.5
        const ix = (cx - centerX - panX) / oldW
        const iy = (cy - centerY - panY) / oldH

        zoom = newZoom
        const newW = photo.sourceSize.width * fitScale * newZoom
        const newH = photo.sourceSize.height * fitScale * newZoom
        const newCenterX = (root.width - newW) * 0.5
        const newCenterY = (root.height - newH) * 0.5
        panX = cx - newCenterX - ix * newW
        panY = cy - newCenterY - iy * newH
        clampPan()
    }

    onWidthChanged: updateFitScale()
    onHeightChanged: updateFitScale()

    Image {
        id: photo
        source: controller.imagePath.length > 0 ? ("file://" + controller.imagePath) : ""
        fillMode: Image.PreserveAspectFit
        width: sourceSize.width > 0 ? sourceSize.width * root.displayScale : 0
        height: sourceSize.height > 0 ? sourceSize.height * root.displayScale : 0
        x: (root.width - width) * 0.5 + root.panX
        y: (root.height - height) * 0.5 + root.panY

        onStatusChanged: {
            if (status === Image.Ready)
                root.resetView()
        }
        onSourceChanged: {
            if (source.length === 0) {
                root.zoom = 1.0
                root.panX = 0
                root.panY = 0
            }
        }
    }

    Repeater {
        model: controller.correspondenceTable
        delegate: Item {
            required property var modelData
            readonly property real sx: photo.width > 0 && photo.sourceSize.width > 0
                ? photo.width / photo.sourceSize.width : 1
            readonly property real sy: photo.height > 0 && photo.sourceSize.height > 0
                ? photo.height / photo.sourceSize.height : 1
            readonly property real markerSize: modelData.selected ? 20 : (modelData.editable ? 16 : 14)
            x: photo.x + modelData.u * sx - markerSize * 0.5
            y: photo.y + modelData.v * sy - markerSize * 0.5
            width: markerSize
            height: markerSize
            z: modelData.selected ? 3 : 2

            Rectangle {
                anchors.fill: parent
                radius: width / 2
                color: modelData.pendingPick
                    ? "#b326d9ff"
                    : (modelData.editable
                        ? (modelData.selected ? "#ccffcc00" : "#b3ff6600")
                        : "#8c8c8c8c")
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
                ? "3D point picked — Shift+click the matching spot on the image"
                : (controller.pendingCorrespondence2d && !controller.pendingCorrespondence3d
                    ? "Image point picked — Shift+click the matching 3D point in the viewer"
                    : "Completing correspondence…")
        }
    }

    Rectangle {
        id: reticle
        width: root.reticleOuterSize
        height: root.reticleOuterSize
        radius: width / 2
        color: "transparent"
        border.color: root.reticleColor
        border.width: 2
        visible: controller.mainWorkTab === 1 && pickMouseArea.containsMouse
        z: 5
        x: pickMouseArea.mouseX - width / 2
        y: pickMouseArea.mouseY - height / 2
        Rectangle {
            anchors.centerIn: parent
            width: 2
            height: parent.height + root.reticleLineExtension * 2
            color: root.reticleColor
        }
        Rectangle {
            anchors.centerIn: parent
            width: parent.width + root.reticleLineExtension * 2
            height: 2
            color: root.reticleColor
        }
    }

    HoverHandler {
        id: hover
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
    }

    WheelHandler {
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onWheel: (event) => {
            if (event.angleDelta.y < 0 && root.zoom <= root.minZoom)
                return
            const factor = event.angleDelta.y > 0 ? 1.1 : 0.9
            root.zoomAt(factor, event.x, event.y)
            event.accepted = true
        }
    }

    DragHandler {
        target: null
        acceptedButtons: Qt.LeftButton
        onActiveChanged: {
            if (active) {
                panDragStartX = root.panX
                panDragStartY = root.panY
            }
        }
        onTranslationChanged: {
            root.panX = panDragStartX + translation.x
            root.panY = panDragStartY + translation.y
            root.clampPan()
        }
    }

    MouseArea {
        id: pickMouseArea
        anchors.fill: parent
        z: 20
        hoverEnabled: true
        enabled: controller.pickingAllowed && controller.mainWorkTab === 1
        cursorShape: enabled ? Qt.BlankCursor : Qt.ArrowCursor
        onClicked: (mouse) => {
            if (!(mouse.modifiers & Qt.ShiftModifier))
                return
            const uv = root.imagePixelFromView(mouse.x, mouse.y)
            if (!uv)
                return
            controller.pickImageCorrespondence(uv.u, uv.v)
        }
    }
}
