import QtQuick 2.10
import QtQuick.Controls 2.6
import QtQuick.Dialogs 1.2


Dialog {
    //visible: true
    title: "Focus entry missing"

    standardButtons: StandardButton.Apply | StandardButton.Cancel

    onApply: {automator.addMissingEntry(Number.fromLocaleString(Qt.locale("ru_RU"), input.text))}

    FocusScope {
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

            validator: DoubleValidator{}
        }
    }
}
