// Dual-handle slider to set colormap min/max with live preview on the range or point cloud.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    id: root
    spacing: 4

    required property real dataMin
    required property real dataMax
    property real rangeMin: dataMin
    property real rangeMax: dataMax

    signal rangeLive(real minValue, real maxValue)
    signal rangeCommitted(real minValue, real maxValue)

    readonly property bool dragging: colormapSlider.first.pressed || colormapSlider.second.pressed

    /// JET RGB for normalized scalar t in [0, 1] (left = low, right = high; matches psiviz shader).
    function jetColor(t) {
        const clamped = Math.max(0, Math.min(1, t))
        const ch = (c) => Math.max(0, Math.min(1, 1.5 - Math.abs(4 * clamped - c)))
        const r = Math.round(ch(3) * 255)
        const g = Math.round(ch(2) * 255)
        const b = Math.round(ch(1) * 255)
        return Qt.rgba(r / 255, g / 255, b / 255, 1)
    }

    function span() {
        const lo = Math.min(dataMin, dataMax)
        const hi = Math.max(dataMin, dataMax)
        return Math.max(hi - lo, 1e-6)
    }

    function emitLive() {
        root.rangeLive(colormapSlider.first.value, colormapSlider.second.value)
    }

    function commitIfReleased() {
        if (!colormapSlider.first.pressed && !colormapSlider.second.pressed) {
            root.rangeCommitted(colormapSlider.first.value, colormapSlider.second.value)
        }
    }

    RowLayout {
        Layout.fillWidth: true
        spacing: 6

        Label {
            text: qsTr("Low")
            font.pixelSize: 10
            opacity: 0.8
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 14
            radius: 2
            color: "transparent"
            border.color: "#909090"
            border.width: 1
            clip: true

            Canvas {
                id: jetLegend
                anchors.fill: parent
                anchors.margins: 1

                onWidthChanged: requestPaint()
                onHeightChanged: requestPaint()
                Component.onCompleted: requestPaint()

                onPaint: {
                    const ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    const w = Math.max(1, Math.floor(width))
                    const h = Math.max(1, Math.floor(height))
                    for (let x = 0; x < w; ++x) {
                        const t = w <= 1 ? 0 : x / (w - 1)
                        ctx.fillStyle = root.jetColor(t)
                        ctx.fillRect(x, 0, 1, h)
                    }
                }
            }
        }

        Label {
            text: qsTr("High")
            font.pixelSize: 10
            opacity: 0.8
        }
    }

    RangeSlider {
        id: colormapSlider
        Layout.fillWidth: true
        from: Math.min(dataMin, dataMax)
        to: Math.max(dataMin, dataMax)
        stepSize: root.span() / 200.0
        first.value: Math.max(from, Math.min(to, root.rangeMin))
        second.value: Math.max(from, Math.min(to, root.rangeMax))

        first.onValueChanged: {
            if (first.pressed || second.pressed) {
                root.emitLive()
            }
        }
        second.onValueChanged: {
            if (first.pressed || second.pressed) {
                root.emitLive()
            }
        }
        first.onPressedChanged: root.commitIfReleased()
        second.onPressedChanged: root.commitIfReleased()
    }

    RowLayout {
        Layout.fillWidth: true
        Label {
            text: colormapSlider.first.value.toFixed(3)
            font.pixelSize: 11
        }
        Item { Layout.fillWidth: true }
        Label {
            text: colormapSlider.second.value.toFixed(3)
            font.pixelSize: 11
        }
    }
}
