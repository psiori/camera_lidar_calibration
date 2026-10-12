import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Psiviz

ApplicationWindow {
    id: root
    width: 1280
    height: 800
    visible: true
    title: "Camera-LiDAR Calibration"

    MaskEditorWindow {
        id: maskEditor
    }

    readonly property int tooltipDelayMs: 1000

    Connections {
        target: controller
        function onPcdCatalogChanged() {
            Qt.callLater(controller.refreshPreviewInViewer)
        }
        function onScanTimeChanged() {
            timeUnit.currentIndex = controller.timeUnit
            timeOrigin.currentIndex = controller.timeOrigin
            scanDuration.text = controller.scanDuration > 0 ? String(controller.scanDuration) : ""
        }
        function onSessionLoaded() {
            timeUnit.currentIndex = controller.timeUnit
            timeOrigin.currentIndex = controller.timeOrigin
            scanDuration.text = controller.scanDuration > 0 ? String(controller.scanDuration) : ""
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        Label {
            text: "Camera-LiDAR calibration"
            font.bold: true
            font.pixelSize: 18
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            BusyIndicator {
                id: busySpinner
                running: controller.busyWorking
                visible: controller.busyWorking
                Layout.preferredWidth: visible ? 28 : 0
                Layout.preferredHeight: 28
            }
            Label {
                Layout.fillWidth: true
                text: controller.statusMessage
            }
        }

        ButtonGroup {
            id: tabGroup
        }

        RowLayout {
            id: tabSegment
            Layout.fillWidth: true
            spacing: 4

            Button {
                id: calibTabBtn
                ButtonGroup.group: tabGroup
                text: "Calibration"
                checkable: true
                checked: true
                Layout.fillWidth: true
                Layout.preferredHeight: 36
                onClicked: {
                    tabStack.currentIndex = 0
                    controller.inspectionMode = false
                }
                background: Rectangle {
                    radius: 6
                    color: calibTabBtn.checked ? "#3d5a80" : "#d8d8dc"
                    border.color: calibTabBtn.checked ? "#2c4460" : "#a8a8b0"
                    border.width: calibTabBtn.checked ? 2 : 1
                }
                contentItem: Text {
                    text: calibTabBtn.text
                    font.bold: calibTabBtn.checked
                    color: calibTabBtn.checked ? "#ffffff" : "#303038"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
            Button {
                id: inspectTabBtn
                ButtonGroup.group: tabGroup
                text: "Inspection"
                checkable: true
                checked: false
                Layout.fillWidth: true
                Layout.preferredHeight: 36
                onClicked: {
                    tabStack.currentIndex = 1
                    controller.inspectionMode = true
                }
                background: Rectangle {
                    radius: 6
                    color: inspectTabBtn.checked ? "#3d5a80" : "#d8d8dc"
                    border.color: inspectTabBtn.checked ? "#2c4460" : "#a8a8b0"
                    border.width: inspectTabBtn.checked ? 2 : 1
                }
                contentItem: Text {
                    text: inspectTabBtn.text
                    font.bold: inspectTabBtn.checked
                    color: inspectTabBtn.checked ? "#ffffff" : "#303038"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }

        SplitView {
            id: workSplit
            orientation: Qt.Horizontal
            Layout.fillWidth: true
            Layout.fillHeight: true

            StackLayout {
                id: tabStack
                SplitView.minimumWidth: 280
                SplitView.preferredWidth: workSplit.width > 0 ? workSplit.width * 0.5 : 400
                currentIndex: 0
                onCurrentIndexChanged: {
                    calibTabBtn.checked = (currentIndex === 0)
                    inspectTabBtn.checked = (currentIndex === 1)
                    controller.inspectionMode = (currentIndex === 1)
                }

                ScrollView {
                    id: calibControlsScroll
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    contentWidth: availableWidth

                    ColumnLayout {
                        width: calibControlsScroll.availableWidth
                        spacing: 4

                        Expansion {
                            Layout.fillWidth: true
                            title: "Dataset"
                            expanded: true
                            ColumnLayout {
                                Layout.fillWidth: true
                                RowLayout {
                                    Layout.fillWidth: true
                                    TextField {
                                        Layout.fillWidth: true
                                        text: controller.imagePath
                                        placeholderText: "Image path"
                                        onEditingFinished: controller.imagePath = text
                                        ToolTip.text: "Path to the camera image for this dataset (used for projection and NID alignment)."
                                        ToolTip.delay: root.tooltipDelayMs
                                    }
                                    Button {
                                        text: "…"
                                        onClicked: controller.browseImageFile()
                                    }
                                }
                                RowLayout {
                                    Layout.fillWidth: true
                                    TextField {
                                        Layout.fillWidth: true
                                        text: controller.pcdDirectory
                                        placeholderText: "PCD directory or file"
                                        onEditingFinished: controller.pcdDirectory = text
                                        ToolTip.text: "Directory of structured PCD scans, or a single .pcd file."
                                        ToolTip.delay: root.tooltipDelayMs
                                    }
                                    Button {
                                        text: "…"
                                        onClicked: controller.browsePcdDirectory()
                                    }
                                }
                                RowLayout {
                                    Layout.fillWidth: true
                                    TextField {
                                        Layout.fillWidth: true
                                        text: controller.intrinsicsPath
                                        placeholderText: "Intrinsics YAML"
                                        readOnly: true
                                        ToolTip.text: "OpenCV camera intrinsics YAML (FileStorage)."
                                        ToolTip.delay: root.tooltipDelayMs
                                    }
                                    Button {
                                        text: "…"
                                        onClicked: controller.browseIntrinsicsFile()
                                    }
                                }
                                Label {
                                    text: "PCD files: " + controller.pcdFileCount
                                         + (controller.fusionBlocked ? " (geometry error)" : "")
                                    Layout.fillWidth: true
                                }
                            }
                        }

                        Expansion {
                            Layout.fillWidth: true
                            title: "Global mask"
                            expanded: false
                            ColumnLayout {
                                Layout.fillWidth: true
                                RowLayout {
                                    Layout.fillWidth: true
                                    Button {
                                        text: "Edit mask…"
                                        onClicked: maskEditor.show()
                                        ToolTip.text: "Open a resizable window to draw the global mask on the range image."
                                        ToolTip.delay: root.tooltipDelayMs
                                    }
                                    Button { text: "Load mask"; onClicked: controller.loadMask() }
                                    Button { text: "Browse mask…"; onClicked: controller.browseMaskFile() }
                                    Button { text: "Save mask"; onClicked: controller.saveMask() }
                                    Button { text: "Clear mask"; onClicked: controller.clearMask() }
                                }
                                Label {
                                    Layout.fillWidth: true
                                    wrapMode: Text.WordWrap
                                    text: controller.maskPath.length > 0 ? ("Mask: " + controller.maskPath)
                                        : "Use Edit mask… to author a polygon on the range image."
                                }
                            }
                        }

                        Expansion {
                            Layout.fillWidth: true
                            title: "Fusion"
                            expanded: false
                            ColumnLayout {
                                Layout.fillWidth: true
                                RowLayout {
                                    Label { text: "Voxel (m)" }
                                    TextField {
                                        implicitWidth: 72
                                        text: String(controller.fusionVoxelResolution)
                                        onEditingFinished: controller.fusionVoxelResolution = parseFloat(text)
                                    }
                                    Label { text: "Threads" }
                                    SpinBox {
                                        from: 1
                                        to: Math.max(64, controller.fusionNumThreads)
                                        value: controller.fusionNumThreads
                                        onValueModified: controller.fusionNumThreads = value
                                    }
                                }
                                RowLayout {
                                    Label { text: "Max corr. (m)" }
                                    TextField {
                                        Layout.fillWidth: true
                                        text: String(controller.fusionMaxCorrespondence)
                                        onEditingFinished: controller.fusionMaxCorrespondence = parseFloat(text)
                                    }
                                }
                            }
                        }

                        Expansion {
                            Layout.fillWidth: true
                            title: "Scan time"
                            expanded: false
                            ColumnLayout {
                                Label { text: "Time unit"; Layout.fillWidth: true }
                                ComboBox {
                                    id: timeUnit
                                    Layout.fillWidth: true
                                    model: ["Seconds", "Nanoseconds"]
                                    currentIndex: controller.timeUnit
                                    onActivated: controller.timeUnit = currentIndex
                                }
                                Label { text: "Time origin"; Layout.fillWidth: true }
                                ComboBox {
                                    id: timeOrigin
                                    Layout.fillWidth: true
                                    model: ["FirstPoint", "ScanStart"]
                                    currentIndex: controller.timeOrigin
                                    onActivated: controller.timeOrigin = currentIndex
                                }
                                RowLayout {
                                    Label { text: "Scan duration (s)" }
                                    TextField {
                                        id: scanDuration
                                        Layout.fillWidth: true
                                        text: controller.scanDuration > 0 ? controller.scanDuration : ""
                                        placeholderText: "required for ScanStart"
                                        onEditingFinished: controller.scanDuration = parseFloat(text)
                                    }
                                }
                            }
                        }

                        Expansion {
                            Layout.fillWidth: true
                            title: "Actions"
                            expanded: false
                            ColumnLayout {
                                Layout.fillWidth: true
                                RowLayout {
                                    Button {
                                        text: "Run fusion"
                                        enabled: !controller.fusionBlocked && !controller.busyWorking
                                        onClicked: controller.runFusion()
                                    }
                                    Button {
                                        text: "Run alignment"
                                        enabled: !controller.busyWorking
                                        onClicked: controller.runAlignment()
                                    }
                                }
                                RowLayout {
                                    Label { text: "Points: " + controller.pointCount }
                                    Button {
                                        text: "Export fused PCD"
                                        enabled: controller.pointCount > 0
                                        onClicked: controller.exportFusedPcd()
                                    }
                                }
                            }
                        }

                        Expansion {
                            Layout.fillWidth: true
                            title: "Initial guess"
                            expanded: false
                            ColumnLayout {
                                Layout.fillWidth: true
                                Label {
                                    Layout.fillWidth: true
                                    wrapMode: Text.WordWrap
                                    text: "Shift+click a 3D point, then click the matching pixel on the camera image. "
                                        + "Use the gizmo for coarse alignment; estimate refines from 3+ pairs."
                                }
                                Label {
                                    text: "Correspondences: " + controller.correspondenceCount
                                        + (controller.pendingCorrespondence3d ? " (pick image…)" : "")
                                }
                                RowLayout {
                                    Button {
                                        text: "Estimate from correspondences"
                                        enabled: controller.correspondenceCount >= 3
                                        onClicked: controller.estimateInitialGuessFromCorrespondences()
                                    }
                                    Button {
                                        text: "Clear"
                                        onClicked: controller.clearCorrespondences()
                                    }
                                }
                                Item {
                                    Layout.fillWidth: true
                                    Layout.preferredHeight: 120
                                    visible: controller.imagePath.length > 0
                                    Image {
                                        id: corrImage
                                        anchors.fill: parent
                                        fillMode: Image.PreserveAspectFit
                                        source: controller.imagePath.length > 0 ? ("file://" + controller.imagePath) : ""
                                        MouseArea {
                                            anchors.fill: parent
                                            enabled: controller.pendingCorrespondence3d
                                            onClicked: function(mouse) {
                                                if (corrImage.sourceSize.width <= 0) return
                                                const sx = corrImage.paintedWidth / corrImage.sourceSize.width
                                                const sy = corrImage.paintedHeight / corrImage.sourceSize.height
                                                const ox = (corrImage.width - corrImage.paintedWidth) * 0.5
                                                const oy = (corrImage.height - corrImage.paintedHeight) * 0.5
                                                const u = (mouse.x - ox) / sx
                                                const v = (mouse.y - oy) / sy
                                                controller.pickImageCorrespondence(u, v)
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                ScrollView {
                    id: inspectScroll
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    contentWidth: availableWidth

                    ColumnLayout {
                        width: inspectScroll.availableWidth
                        spacing: 8
                        Label {
                            Layout.fillWidth: true
                            wrapMode: Text.WordWrap
                            text: "Inspection datasets are stored separately in the session. "
                                + "Fusion uses the global mask; overlay uses the saved extrinsic (not NID)."
                        }
                    Expansion {
                        Layout.fillWidth: true
                        title: "Dataset"
                        expanded: true
                        ColumnLayout {
                            Layout.fillWidth: true
                            RowLayout {
                                Layout.fillWidth: true
                                TextField {
                                    Layout.fillWidth: true
                                    text: controller.imagePath
                                    placeholderText: "Image path"
                                    onEditingFinished: controller.imagePath = text
                                }
                                Button { text: "…"; onClicked: controller.browseImageFile() }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                TextField {
                                    Layout.fillWidth: true
                                    text: controller.pcdDirectory
                                    placeholderText: "PCD directory or file"
                                    onEditingFinished: controller.pcdDirectory = text
                                }
                                Button { text: "…"; onClicked: controller.browsePcdDirectory() }
                            }
                            Button {
                                text: "Fuse inspection scans"
                                enabled: !controller.fusionBlocked && !controller.busyWorking
                                onClicked: controller.runFusion()
                            }
                            Button {
                                text: "Export colored PCD"
                                enabled: controller.pointCount > 0
                                onClicked: controller.exportInspectionColoredPcd()
                            }
                        }
                    }
                    }
                }
            }

            FocusScope {
                id: viewerPane
                SplitView.fillWidth: true
                SplitView.minimumWidth: 320
                SplitView.preferredWidth: workSplit.width > 0 ? workSplit.width * 0.5 : 400
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
                    gizmoEnabled: !controller.inspectionMode
                    focus: viewerHover.hovered
                    activeFocusOnTab: true
                    Keys.onPressed: (event) => {
                        if (!viewerHover.hovered || viewerOverlay.keysBlocked) {
                            return
                        }
                        if (cloudView.handleNavigationKey(event.key, event.modifiers)) {
                            event.accepted = true
                        }
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
        }

        RowLayout {
            Layout.fillWidth: true
            TextField {
                id: sessionDir
                placeholderText: "Session directory"
                Layout.fillWidth: true
                text: controller.sessionDirectory
                onEditingFinished: controller.sessionDirectory = text
            }
            Button { text: "…"; onClicked: controller.browseSessionDirectory() }
            Button {
                text: "Load"
                onClicked: controller.loadSession(sessionDir.text.length > 0 ? sessionDir.text : controller.sessionDirectory)
            }
            Button {
                text: "Save"
                onClicked: controller.saveSession(sessionDir.text.length > 0 ? sessionDir.text : controller.sessionDirectory)
            }
        }
    }
}
