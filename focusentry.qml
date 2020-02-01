import QtQuick 2.10
import QtQuick.Controls 2.6
import QtQuick.Dialogs 1.2


Dialog {
    visible: true
    title: "Focus entry missing"

    standardButtons: StandardButton.Apply | StandardButton.Cancel

    onApply: {automator.addMissingEntry(parseFloat(input.text))}

    contentItem: FocusScope {
        width: 96; height: 30
        Rectangle {
            anchors.fill: parent
            color: "lightsteelblue"
            border.color: "gray"
        }
        TextInput {
            id: input
            anchors.fill: parent
            focus: true
            font.pointSize: 20

            validator: DoubleValidator{bottom:-20; top:20}
        }
    }
}
