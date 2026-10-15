// Reusable pan/zoom image viewport: fit-to-view, bounded wheel zoom, drag pan, click-to-pick mapping.
import QtQuick

Item {
    id: root
    clip: true

    /// Image URL (e.g. file:// path).
    property url source

    /// When false, wheel zoom and drag pan are disabled.
    property bool navigationEnabled: true

    /// When false, tap-to-click and reticle are disabled.
    property bool clickEnabled: true

    /// Maximum zoom multiplier on top of the fit-to-view scale.
    property real maxZoom: 12.0

    /// Base for exponential wheel zoom; values > 1.0 zoom in per wheel step (tune for feel).
    property real wheelZoomBase: 1.08

    /// Draw a circular reticle and crosshair at the pointer when hovered.
    property bool showReticle: true

    property color reticleColor: "#00e5ff"

    /// Stroke width for the reticle ring and cross lines (device pixels).
    property real reticleLineWidth: 1.0

    /// Diameter of the reticle ring centered on the pointer.
    property real reticleOuterSize: 24.0

    /// How far crosshair lines extend beyond the ring on each side.
    property real reticleLineExtension: 8.0

    readonly property real minZoom: 1.0

    property real fitScale: 1.0
    property real zoom: 1.0
    property real panX: 0
    property real panY: 0

    property real _panDragStartX: 0
    property real _panDragStartY: 0

    readonly property real displayScale: fitScale * zoom

    /// Top-left of the displayed image in view coordinates.
    readonly property real imageX: photo.x
    readonly property real imageY: photo.y
    readonly property real imageDisplayWidth: photo.width
    readonly property real imageDisplayHeight: photo.height
    readonly property size nativeSourceSize: photo.sourceSize

    signal viewClicked(real viewX, real viewY)
    signal imageClicked(real u, real v)

    /// Recompute letterbox fit and reset zoom/pan (e.g. after a new image loads).
    function resetView() {
        updateFitScale()
        zoom = 1.0
        panX = 0
        panY = 0
        clampPan()
    }

    /// Map view coordinates to native image pixels; returns null if outside the image.
    function viewToImagePixel(viewX, viewY) {
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

    /// Map native image pixel (u, v) to view coordinates at the pixel center.
    function imageToView(u, v) {
        if (photo.sourceSize.width <= 0 || photo.width <= 0 || photo.height <= 0)
            return null
        const sx = photo.width / photo.sourceSize.width
        const sy = photo.height / photo.sourceSize.height
        return {
            x: photo.x + u * sx,
            y: photo.y + v * sy
        }
    }

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

    /// Keep the image covering the viewport when zoomed in; center when smaller than the view.
    function clampPan() {
        if (photo.width <= 0 || photo.height <= 0)
            return
        const centerX = (root.width - photo.width) * 0.5
        const centerY = (root.height - photo.height) * 0.5

        if (photo.width <= root.width)
            panX = 0
        else {
            const maxPanX = -centerX
            const minPanX = root.width - photo.width - centerX
            panX = Math.min(maxPanX, Math.max(minPanX, panX))
        }

        if (photo.height <= root.height)
            panY = 0
        else {
            const maxPanY = -centerY
            const minPanY = root.height - photo.height - centerY
            panY = Math.min(maxPanY, Math.max(minPanY, panY))
        }
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
        const ix = oldW > 0 ? (cx - centerX - panX) / oldW : 0.5
        const iy = oldH > 0 ? (cy - centerY - panY) / oldH : 0.5

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
        source: root.source
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
            if (source === "" || source === null) {
                root.zoom = 1.0
                root.panX = 0
                root.panY = 0
            }
        }
    }

    Item {
        id: reticle
        visible: root.showReticle && root.clickEnabled && cursorArea.containsMouse
        z: 100
        width: root.reticleOuterSize
        height: root.reticleOuterSize
        x: cursorArea.mouseX - width * 0.5
        y: cursorArea.mouseY - height * 0.5

        Rectangle {
            anchors.fill: parent
            radius: width * 0.5
            color: "transparent"
            border.color: root.reticleColor
            border.width: root.reticleLineWidth
        }
        Rectangle {
            anchors.centerIn: parent
            width: root.reticleLineWidth
            height: parent.height + root.reticleLineExtension * 2
            color: root.reticleColor
        }
        Rectangle {
            anchors.centerIn: parent
            width: parent.width + root.reticleLineExtension * 2
            height: root.reticleLineWidth
            color: root.reticleColor
        }
    }

    readonly property bool hideSystemCursor: root.showReticle
        && (root.navigationEnabled || root.clickEnabled)

    HoverHandler {
        id: hover
        enabled: root.navigationEnabled || root.clickEnabled
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        cursorShape: root.hideSystemCursor ? Qt.BlankCursor : Qt.ArrowCursor
    }

    // Tracks pointer for reticle and enforces blank system cursor (HoverHandler alone is unreliable on macOS).
    MouseArea {
        id: cursorArea
        anchors.fill: parent
        z: 200
        hoverEnabled: true
        acceptedButtons: Qt.NoButton
        propagateComposedEvents: true
        enabled: root.navigationEnabled || root.clickEnabled
        cursorShape: root.hideSystemCursor ? Qt.BlankCursor : Qt.ArrowCursor
    }

    WheelHandler {
        enabled: root.navigationEnabled
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onWheel: (event) => {
            const delta = event.angleDelta.y
            if (delta === 0)
                return
            if (delta < 0 && root.zoom <= root.minZoom)
                return
            const steps = delta / 120.0
            const factor = Math.pow(root.wheelZoomBase, steps)
            root.zoomAt(factor, event.x, event.y)
            event.accepted = true
        }
    }

    DragHandler {
        enabled: root.navigationEnabled
        target: null
        acceptedButtons: Qt.LeftButton
        dragThreshold: 4
        onActiveChanged: {
            if (active) {
                root._panDragStartX = root.panX
                root._panDragStartY = root.panY
            }
        }
        onTranslationChanged: {
            root.panX = root._panDragStartX + translation.x
            root.panY = root._panDragStartY + translation.y
            root.clampPan()
        }
    }

    TapHandler {
        id: pickTap
        enabled: root.clickEnabled
        acceptedButtons: Qt.LeftButton
        onTapped: (eventPoint) => {
            const px = eventPoint.position.x
            const py = eventPoint.position.y
            root.viewClicked(px, py)
            const uv = root.viewToImagePixel(px, py)
            if (uv)
                root.imageClicked(uv.u, uv.v)
        }
    }
}
