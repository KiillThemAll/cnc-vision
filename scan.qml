import QtQuick 2.10
import QtQuick.Window 2.10
import QtQuick.Controls 2.6
import QtQuick.Layouts 1.12

Window {
    id: scanWindow
    visible: true
    width: 1920
    height: 1080
    title: qsTr("Hello World")
    color: "#161616"

    RowLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        spacing: 10
        Layout.maximumHeight: 50
        Text {
            text: "Surface size (X, Y, step)"
            color: "gray"
            font.pointSize: 25
        }

        FocusScope {
            width: 96; height: 30
            Rectangle {
                anchors.fill: parent
                color: "lightsteelblue"
                border.color: "gray"

            }

            TextInput {
                id: input1
                anchors.fill: parent
                focus: true
                font.pointSize: 20

                text: "1500"

                validator: IntValidator{bottom:0; top:1525}
            }
        }



        FocusScope {
            width: 96; height: 30
            Rectangle {
                anchors.fill: parent
                color: "lightsteelblue"
                border.color: "gray"

            }

            TextInput {
                id: input2
                anchors.fill: parent
                focus: true
                font.pointSize: 20

                text: "1500"

                validator: IntValidator{bottom:0; top:1525}
            }
        }

        FocusScope {
            width: 96; height: 30
            Rectangle {
                anchors.fill: parent
                color: "lightsteelblue"
                border.color: "gray"

            }

            TextInput {
                id: input3
                anchors.fill: parent
                focus: true
                font.pointSize: 20

                text: "20"

                validator: IntValidator{bottom:0; top:1525}
            }
        }


        Button {
            text: "Scan"

            onClicked: automator.scanSurface(parseInt(input1.text, 10),parseInt(input2.text, 10),parseInt(input3.text, 10))
        }


    }

}
