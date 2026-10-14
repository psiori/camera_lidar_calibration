// Collapsible section wrapper for dense parameter groups in the main form.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    id: root
    property string title: ""
    property bool expanded: true
    default property alias content: body.data
    spacing: 4

    Button {
        Layout.fillWidth: true
        Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
        flat: true
        text: root.title + (root.expanded ? " ▾" : " ▸")
        onClicked: root.expanded = !root.expanded
        contentItem: Text {
            text: parent.text
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
            color: parent.palette.windowText
        }
    }

    ColumnLayout {
        id: body
        Layout.fillWidth: true
        visible: root.expanded
        spacing: 4
    }
}
