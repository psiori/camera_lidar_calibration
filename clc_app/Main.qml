import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: root
    width: 1400
    height: 900
    visible: true
    title: "Camera-LiDAR Calibration"

    MaskEditorWindow { id: maskEditor }

    AddDatasetDialog {
        id: addDatasetDialog
        parent: root
    }

    readonly property int tooltipDelayMs: 1000

    component TipRow: RowLayout {
        property string tip: ""
        HoverHandler { id: tipHover }
        ToolTip.visible: tipHover.hovered && tip.length > 0
        ToolTip.text: tip
        ToolTip.delay: root.tooltipDelayMs
    }

    Connections {
        target: controller
        function onPcdCatalogChanged() { Qt.callLater(controller.refreshPreviewInViewer) }
        function onScanTimeChanged() {
            timeUnit.currentIndex = controller.timeUnit
            timeOrigin.currentIndex = controller.timeOrigin
            scanDuration.text = controller.scanDuration > 0 ? String(controller.scanDuration) : ""
        }
        function onSessionLoaded() { onScanTimeChanged() }
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
            BusyIndicator {
                running: controller.busyWorking
                visible: controller.busyWorking
                Layout.preferredWidth: visible ? 28 : 0
            }
            Label {
                Layout.fillWidth: true
                text: controller.statusMessage
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 4
            Repeater {
                model: ["Datasets", "Initial alignment", "Calibrate", "Inspection"]
                delegate: Button {
                    required property int index
                    required property string modelData
                    property bool selected: controller.mainWorkTab === index
                    text: modelData
                    checkable: true
                    checked: selected
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    onClicked: controller.setMainWorkTab(index)
                    background: Rectangle {
                        radius: 6
                        color: parent.selected ? "#3d5a80" : "#d8d8dc"
                        border.color: parent.selected ? "#2c4460" : "#a8a8b0"
                        border.width: parent.selected ? 2 : 1
                    }
                    contentItem: Text {
                        text: parent.text
                        font.bold: parent.selected
                        color: parent.selected ? "#ffffff" : "#303038"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            visible: controller.mainWorkTab !== 3
            spacing: 6
            Label { text: "Calibration datasets:" }
            Repeater {
                model: controller.calibrationDatasetIds
                delegate: Button {
                    required property int index
                    required property string modelData
                    text: modelData
                    checkable: true
                    checked: controller.activeCalibrationDatasetIndex === index
                    enabled: controller.mainWorkTab === 0 || controller.isCalibrationDatasetReady(index)
                    ToolTip.visible: hovered && !enabled
                    ToolTip.text: controller.calibrationDatasetTooltip(index)
                    ToolTip.delay: root.tooltipDelayMs
                    onClicked: controller.setActiveCalibrationDatasetIndex(index)
                }
            }
            Button {
                text: "Add…"
                visible: controller.mainWorkTab === 0
                onClicked: addDatasetDialog.open()
            }
            Button {
                text: "Remove"
                visible: controller.mainWorkTab === 0
                onClicked: controller.removeActiveCalibrationDataset()
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 4

            StackLayout {
                id: workPanels
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: controller.mainWorkTab

                // Datasets — controls only (3D view is shared on the right)
                ScrollView {
                    id: datasetsScroll
                    clip: true
                    ColumnLayout {
                        width: datasetsScroll.availableWidth
                        spacing: 6
                        Label {
                            Layout.fillWidth: true
                            wrapMode: Text.WordWrap
                            text: "Image: " + controller.imagePath + "\nPCD: " + controller.pcdDirectory
                        }
                        Label {
                            text: "PCD files: " + controller.pcdFileCount
                                 + (controller.fusionBlocked ? " (geometry error)" : "")
                        }
                        RowLayout {
                            Label { text: "Intrinsics" }
                            TextField {
                                Layout.fillWidth: true
                                text: controller.intrinsicsPath
                                readOnly: true
                            }
                        }
                        Label {
                            Layout.fillWidth: true
                            wrapMode: Text.WordWrap
                            visible: controller.activeDatasetIsSingleScan
                            text: "Single scan loaded: scan-time and multi-scan fusion options are disabled; use voxel size and Run fusion for passthrough."
                            font.italic: true
                        }
                        GroupBox {
                            title: "Fusion"
                            Layout.fillWidth: true
                            ColumnLayout {
                                TipRow {
                                    tip: "Output voxel size in meters (default 0.01). Downsampling after fusion or single-scan passthrough."
                                    Label { text: "Voxel (m)" }
                                    TextField {
                                        text: String(controller.fusionVoxelResolution)
                                        onEditingFinished: controller.fusionVoxelResolution = parseFloat(text)
                                    }
                                }
                                TipRow {
                                    tip: "OpenMP threads for multi-scan CT-GICP fusion (default 15)."
                                    Label {
                                        text: "Threads"
                                        opacity: controller.activeDatasetIsSingleScan ? 0.45 : 1
                                    }
                                    SpinBox {
                                        editable: true
                                        from: 1; to: 64
                                        value: controller.fusionNumThreads
                                        enabled: !controller.activeDatasetIsSingleScan
                                        opacity: enabled ? 1 : 0.45
                                        onValueModified: controller.fusionNumThreads = value
                                    }
                                }
                                TipRow {
                                    tip: "Max correspondence distance in meters for fusion (default 0.5)."
                                    Label {
                                        text: "Max corr. (m)"
                                        opacity: controller.activeDatasetIsSingleScan ? 0.45 : 1
                                    }
                                    TextField {
                                        Layout.fillWidth: true
                                        text: String(controller.fusionMaxCorrespondence)
                                        enabled: !controller.activeDatasetIsSingleScan
                                        opacity: enabled ? 1 : 0.45
                                        onEditingFinished: controller.fusionMaxCorrespondence = parseFloat(text)
                                    }
                                }
                                TipRow {
                                    tip: "Neighbors per point for covariance estimation during fusion preprocessing (default 40; applies to single-scan passthrough and multi-scan CT-GICP)."
                                    Label { text: "K neighbors" }
                                    SpinBox {
                                        editable: true
                                        from: 2; to: 128
                                        value: controller.fusionKNeighbors
                                        onValueModified: controller.fusionKNeighbors = value
                                    }
                                }
                                Button {
                                    text: "Run fusion"
                                    enabled: !controller.fusionBlocked && !controller.busyWorking
                                    onClicked: controller.runFusion()
                                    ToolTip.visible: hovered
                                    ToolTip.text: controller.activeDatasetIsSingleScan
                                        ? "Downsample the single masked scan to the voxel size."
                                        : "Merge scans for the active dataset using the global mask and scan-time settings."
                                    ToolTip.delay: root.tooltipDelayMs
                                }
                            }
                        }
                        GroupBox {
                            title: "Scan time"
                            Layout.fillWidth: true
                            enabled: !controller.activeDatasetIsSingleScan
                            opacity: enabled ? 1 : 0.45
                            ColumnLayout {
                                ComboBox {
                                    id: timeUnit
                                    Layout.fillWidth: true
                                    model: ["Seconds", "Nanoseconds"]
                                    currentIndex: controller.timeUnit
                                    onActivated: controller.timeUnit = currentIndex
                                }
                                ComboBox {
                                    id: timeOrigin
                                    Layout.fillWidth: true
                                    model: ["FirstPoint", "ScanStart"]
                                    currentIndex: controller.timeOrigin
                                    onActivated: controller.timeOrigin = currentIndex
                                }
                                TextField {
                                    id: scanDuration
                                    Layout.fillWidth: true
                                    placeholderText: "Scan duration (s)"
                                    onEditingFinished: controller.scanDuration = parseFloat(text)
                                }
                            }
                        }
                        GroupBox {
                            title: "Global mask"
                            Layout.fillWidth: true
                            RowLayout {
                                Button { text: "Edit mask…"; onClicked: maskEditor.show() }
                                Button { text: "Load"; onClicked: controller.loadMask() }
                                Button { text: "Browse…"; onClicked: controller.browseMaskFile() }
                                Button { text: "Save"; onClicked: controller.saveMask() }
                                Button { text: "Clear"; onClicked: controller.clearMask() }
                            }
                        }
                        Button {
                            text: "Export fused PCD"
                            enabled: controller.pointCount > 0
                            onClicked: controller.exportFusedPcd()
                        }
                    }
                }

                // Initial alignment — image + correspondence (3D view shared)
                SplitView {
                    orientation: Qt.Horizontal
                    SplitView.fillWidth: true
                    CameraImageItem {
                        SplitView.minimumWidth: 240
                        SplitView.preferredWidth: parent.width * 0.5
                    }
                    ScrollView {
                        id: initialAlignScroll
                        SplitView.minimumWidth: 220
                        SplitView.fillWidth: true
                        clip: true
                        ColumnLayout {
                            width: initialAlignScroll.availableWidth
                            spacing: 8
                            Label {
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                                text: "Shift+click a 3D point and/or Shift+click the image (either order). Select a row to re-pick one side."
                            }
                            ComboBox {
                                Layout.fillWidth: true
                                model: ["Manual correspondences"]
                                currentIndex: controller.initialAlignmentMethod
                                onActivated: controller.setInitialAlignmentMethod(currentIndex)
                            }
                            Button {
                                text: "Compute initial alignment"
                                enabled: controller.canComputeInitialAlignment && !controller.busyWorking
                                onClicked: controller.estimateInitialGuessFromCorrespondences()
                                ToolTip.visible: hovered
                                ToolTip.text: "Estimate shared T_camera_lidar from all correspondence pairs (≥3 total across datasets)."
                                ToolTip.delay: root.tooltipDelayMs
                            }
                            Button {
                                text: "Clear active dataset pairs"
                                onClicked: controller.clearCorrespondences()
                            }
                            ListView {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 280
                                clip: true
                                model: controller.correspondenceTable
                                delegate: Rectangle {
                                    required property var modelData
                                    width: ListView.view.width
                                    height: 36
                                    color: modelData.selected ? "#e8f0ff" : (modelData.editable ? "#ffffff" : "#f0f0f0")
                                    opacity: modelData.editable ? 1.0 : 0.75
                                    MouseArea {
                                        anchors.fill: parent
                                        z: -1
                                        enabled: modelData.globalIndex >= 0
                                        onClicked: controller.setSelectedCorrespondenceRow(modelData.globalIndex)
                                    }
                                    RowLayout {
                                        anchors.fill: parent
                                        anchors.margins: 6
                                        Label {
                                            Layout.fillWidth: true
                                            text: {
                                                if (modelData.pendingPick && modelData.pendingKind === "3d") {
                                                    return modelData.datasetId + "  (pending 3D)  x="
                                                        + modelData.x.toFixed(2) + " y=" + modelData.y.toFixed(2)
                                                        + " z=" + modelData.z.toFixed(2)
                                                }
                                                if (modelData.pendingPick) {
                                                    return modelData.datasetId + "  (pending image)  u="
                                                        + modelData.u.toFixed(1) + " v=" + modelData.v.toFixed(1)
                                                }
                                                return modelData.datasetId + "  u=" + modelData.u.toFixed(1)
                                                    + " v=" + modelData.v.toFixed(1)
                                            }
                                        }
                                        Button {
                                            text: "×"
                                            visible: modelData.editable || modelData.pendingPick
                                            onClicked: controller.deleteCorrespondenceRow(modelData.globalIndex)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Calibrate
                ScrollView {
                    id: calibrateScroll
                    clip: true
                    ColumnLayout {
                        width: calibrateScroll.availableWidth
                        spacing: 6
                        Label {
                            wrapMode: Text.WordWrap
                            Layout.fillWidth: true
                            text: "Session-global NID on all ready calibration datasets. Point density: Datasets tab, fusion voxel (default 1 cm)."
                        }
                        GroupBox {
                            title: "NID optimization"
                            Layout.fillWidth: true
                            ColumnLayout {
                                width: parent.width
                                spacing: 4
                                ComboBox {
                                    id: nidRegistration
                                    Layout.fillWidth: true
                                    model: ["BFGS (nid_bfgs)", "Nelder–Mead (nid_nelder_mead)"]
                                    currentIndex: controller.calibrationRegistrationType
                                    onActivated: controller.calibrationRegistrationType = currentIndex
                                    HoverHandler { id: regTipHover }
                                    ToolTip.visible: regTipHover.hovered
                                    ToolTip.text: "BFGS (default) or Nelder–Mead optimizer for each outer NID iteration."
                                    ToolTip.delay: root.tooltipDelayMs
                                }
                                TipRow {
                                    tip: "Histogram bins for mutual information cost (vlcal nid_bins)."
                                    Label { text: "NID bins" }
                                    SpinBox {
                                        editable: true
                                        from: 2; to: 256
                                        value: controller.calibrationNidBins
                                        onValueModified: controller.calibrationNidBins = value
                                    }
                                }
                                TipRow {
                                    tip: "Outer: max refinement loops. Inner: solver steps per outer iteration (BFGS or Nelder–Mead)."
                                    Label { text: "Outer iters" }
                                    SpinBox {
                                        editable: true
                                        from: 1; to: 1000
                                        value: controller.calibrationMaxOuterIterations
                                        onValueModified: controller.calibrationMaxOuterIterations = value
                                    }
                                    Label { text: "Inner iters" }
                                    SpinBox {
                                        editable: true
                                        from: 1; to: 10000
                                        value: controller.calibrationMaxInnerIterations
                                        onValueModified: controller.calibrationMaxInnerIterations = value
                                    }
                                }
                                TipRow {
                                    tip: "Stop outer loop when pose change between iterations is below these thresholds."
                                    Label { text: "Δtrans (m)" }
                                    TextField {
                                        text: String(controller.calibrationDeltaTransThresh)
                                        onEditingFinished: controller.calibrationDeltaTransThresh = parseFloat(text)
                                    }
                                    Label { text: "Δrot (°)" }
                                    TextField {
                                        text: String(controller.calibrationDeltaRotDeg)
                                        onEditingFinished: controller.calibrationDeltaRotDeg = parseFloat(text)
                                    }
                                }
                                CheckBox {
                                    text: "Disable Z-buffer culling"
                                    checked: controller.calibrationDisableZBufferCulling
                                    onToggled: controller.calibrationDisableZBufferCulling = checked
                                    HoverHandler { id: zCullTipHover }
                                    ToolTip.visible: zCullTipHover.hovered
                                    ToolTip.text: "Skip depth-buffer hidden-point removal before NID (more points, slower)."
                                    ToolTip.delay: root.tooltipDelayMs
                                }
                                TipRow {
                                    tip: "OpenMP threads for Nelder–Mead multi-dataset cost only; ignored by BFGS in current vlcal."
                                    Label { text: "Threads" }
                                    SpinBox {
                                        editable: true
                                        from: 1; to: 64
                                        value: controller.calibrationNumThreads
                                        enabled: controller.calibrationRegistrationType === 1
                                        opacity: enabled ? 1 : 0.45
                                        onValueModified: controller.calibrationNumThreads = value
                                    }
                                }
                                GroupBox {
                                    title: "Nelder–Mead"
                                    Layout.fillWidth: true
                                    enabled: controller.calibrationRegistrationType === 1
                                    opacity: enabled ? 1 : 0.45
                                    ColumnLayout {
                                        width: parent.width
                                        TipRow {
                                            tip: "Initial simplex step size for Nelder–Mead (vlcal nelder_mead_init_step)."
                                            Label { text: "Init step" }
                                            TextField {
                                                Layout.fillWidth: true
                                                text: String(controller.calibrationNelderMeadInitStep)
                                                onEditingFinished: controller.calibrationNelderMeadInitStep = parseFloat(text)
                                            }
                                        }
                                        TipRow {
                                            tip: "Nelder–Mead stop when simplex size falls below this (nelder_mead_convergence_criteria)."
                                            Label { text: "Convergence" }
                                            TextField {
                                                Layout.fillWidth: true
                                                text: String(controller.calibrationNelderMeadConvergence)
                                                onEditingFinished: controller.calibrationNelderMeadConvergence = parseFloat(text)
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        Button {
                            text: "Run alignment"
                            enabled: !controller.busyWorking
                            onClicked: controller.runAlignment()
                            ToolTip.visible: hovered
                            ToolTip.text: "Optimize camera–lidar extrinsic (NID) using ready calibration datasets, global mask, and settings above."
                            ToolTip.delay: root.tooltipDelayMs
                        }
                    }
                }

                // Inspection
                ScrollView {
                    id: inspectionScroll
                    clip: true
                    ColumnLayout {
                        width: inspectionScroll.availableWidth
                        Label { wrapMode: Text.WordWrap; Layout.fillWidth: true; text: "Inspection datasets (separate from calibration)." }
                        TextField {
                            Layout.fillWidth: true
                            text: controller.imagePath
                            placeholderText: "Image"
                            onEditingFinished: controller.imagePath = text
                        }
                        TextField {
                            Layout.fillWidth: true
                            text: controller.pcdDirectory
                            placeholderText: "PCD directory"
                            onEditingFinished: controller.pcdDirectory = text
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

            PointCloudViewerPane {
                id: sharedCloudViewer
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.minimumWidth: 280
                Layout.preferredWidth: root.width * (controller.mainWorkTab === 1 ? 0.32 : 0.42)
            }
        }

        RowLayout {
            Layout.fillWidth: true
            TextField {
                id: sessionDir
                Layout.fillWidth: true
                placeholderText: "Session directory"
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
