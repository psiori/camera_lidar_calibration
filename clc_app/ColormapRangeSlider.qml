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
