import QtQuick
import QtQuick.Controls
import Psiviz

FocusScope {
    id: viewerPane
    property alias overlay: viewerOverlay

    focus: true

    HoverHandler {
        id: viewerHover
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        onHoveredChanged: {
            if (hovered) {
                cloudView.forceActiveFocus()
                cloudView.setPointerPosition(point.position.x, point.position.y)
            }
        }
    }

    Timer {
        interval: 16
        running: viewerHover.hovered
        repeat: true
        onTriggered: cloudView.setPointerPosition(viewerHover.point.position.x, viewerHover.point.position.y)
    }

    Shortcut {
        sequences: ["F", "f"]
        context: Qt.ApplicationShortcut
        enabled: viewerHover.hovered && !viewerOverlay.keysBlocked
        onActivated: cloudView.flyToUnderCursor()
    }
    Shortcut {
        sequences: ["+", "Shift+=", "="]
        context: Qt.ApplicationShortcut
        enabled: viewerHover.hovered && !viewerOverlay.keysBlocked
        onActivated: cloudView.bumpPointSize(0.5)
    }
    Shortcut {
        sequences: ["-", "_"]
        context: Qt.ApplicationShortcut
        enabled: viewerHover.hovered && !viewerOverlay.keysBlocked
        onActivated: cloudView.bumpPointSize(-0.5)
    }

    PointCloudScene {
        id: cloudView
        anchors.fill: parent
        focus: viewerHover.hovered
        activeFocusOnTab: true
        Keys.onPressed: (event) => {
            if (!viewerHover.hovered || viewerOverlay.keysBlocked) return
            if (cloudView.handleNavigationKey(event.key, event.modifiers)) event.accepted = true
        }
        Component.onCompleted: Qt.callLater(controller.setViewer, cloudView)
    }

    ViewerOverlay {
        id: viewerOverlay
        cloudView: cloudView
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
    }
}
