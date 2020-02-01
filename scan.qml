import QtQuick 2.10
import QtQuick.Window 2.10
import QtQuick.Controls 2.6
import QtQuick.Layouts 1.12
import QtDataVisualization 1.2

Window {
    id: scanWindow
    visible: true
    width: 1920
    height: 1080
    title: qsTr("Hello World")
    color: "#161616"

    RowLayout {
        id: controls

        anchors.left: parent.left
        width: parent.width
        height: parent.height/10

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

                text: "100"

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

                text: "100"

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

                text: "10"

                validator: IntValidator{bottom:0; top:1525}
            }
        }


        Button {
            text: "Scan"

            onClicked: automator.scanSurface(parseInt(input1.text, 10),parseInt(input2.text, 10),parseInt(input3.text, 10))
        }

        Text {
            font.pointSize: 20
            text: automator.message
        }

        Button {
            text: "Approve scan"
            enabled: automator.scanComplited

            onClicked: automator.approveScan()
        }

        Button {
            text: "Clear scan"

            onClicked: automator.surfaceModel.removeAll()
        }


    }

    /*ListModel {
            id: dataModel
            ListElement{ longitude: "0"; latitude: "0"; height: "124"; }
            ListElement{ longitude: "0"; latitude: "1"; height: "125"; }
            ListElement{ longitude: "0"; latitude: "2"; height: "124"; }
            //! [0]
            ListElement{ longitude: "0"; latitude: "3"; height: "118"; }
            ListElement{ longitude: "0"; latitude: "4"; height: "112"; }
            ListElement{ longitude: "0"; latitude: "5"; height: "111"; }
            ListElement{ longitude: "0"; latitude: "6"; height: "115"; }
            ListElement{ longitude: "0"; latitude: "7"; height: "102"; }
            ListElement{ longitude: "0"; latitude: "8"; height: "115"; }
            ListElement{ longitude: "0"; latitude: "9"; height: "126"; }
            ListElement{ longitude: "0"; latitude: "10"; height: "127"; }
            ListElement{ longitude: "0"; latitude: "11"; height: "127"; }
            ListElement{ longitude: "0"; latitude: "12"; height: "124"; }
            ListElement{ longitude: "0"; latitude: "13"; height: "120"; }
            ListElement{ longitude: "0"; latitude: "14"; height: "117"; }
            ListElement{ longitude: "0"; latitude: "15"; height: "116"; }
            ListElement{ longitude: "0"; latitude: "16"; height: "114"; }
            ListElement{ longitude: "0"; latitude: "17"; height: "112"; }
            ListElement{ longitude: "0"; latitude: "18"; height: "114"; }
            ListElement{ longitude: "0"; latitude: "19"; height: "114"; }
            ListElement{ longitude: "0"; latitude: "20"; height: "112"; }
            ListElement{ longitude: "1"; latitude: "20"; height: "112"; }
            ListElement{ longitude: "1"; latitude: "19"; height: "120"; }
            ListElement{ longitude: "1"; latitude: "18"; height: "120"; }
            ListElement{ longitude: "1"; latitude: "17"; height: "118"; }
            ListElement{ longitude: "1"; latitude: "16"; height: "110"; }
            ListElement{ longitude: "1"; latitude: "15"; height: "105"; }
            ListElement{ longitude: "1"; latitude: "14"; height: "110"; }
            ListElement{ longitude: "1"; latitude: "13"; height: "116"; }
            ListElement{ longitude: "1"; latitude: "12"; height: "117"; }
            ListElement{ longitude: "1"; latitude: "11"; height: "123"; }
            ListElement{ longitude: "1"; latitude: "10"; height: "128"; }
            ListElement{ longitude: "1"; latitude: "9"; height: "131"; }
            ListElement{ longitude: "1"; latitude: "8"; height: "130"; }
            ListElement{ longitude: "1"; latitude: "7"; height: "128"; }
            ListElement{ longitude: "1"; latitude: "6"; height: "122"; }
            ListElement{ longitude: "1"; latitude: "5"; height: "119"; }
            ListElement{ longitude: "1"; latitude: "4"; height: "116"; }
            ListElement{ longitude: "1"; latitude: "3"; height: "113"; }
            ListElement{ longitude: "1"; latitude: "2"; height: "109"; }
            ListElement{ longitude: "1"; latitude: "1"; height: "109"; }
            ListElement{ longitude: "1"; latitude: "0"; height: "109"; }
        }*/

    Item {
        id: surfaceView
        width: parent.width

        anchors.top: controls.bottom
        anchors.left: parent.left
        anchors.bottom: parent.bottom

        ColorGradient {
            id: surfaceGradient
            ColorGradientStop { position: 0.0; color: "darkslategray" }
            ColorGradientStop { id: middleGradient; position: 0.25; color: "peru" }
            ColorGradientStop { position: 1.0; color: "red" }
        }

        Surface3D {
            id: surfacePlot
            width: surfaceView.width
            height: surfaceView.height
            theme: Theme3D {
                type: Theme3D.ThemeStoneMoss
                font.family: "STCaiyun"
                font.pointSize: 35
                //colorStyle: Theme3D.ColorStyleRangeGradient
                //baseGradients: [surfaceGradient]
            }
            shadowQuality: AbstractGraph3D.ShadowQualityNone
            //selectionMode: AbstractGraph3D.SelectionSlice | AbstractGraph3D.SelectionItemAndRow
            selectionMode: AbstractGraph3D.SelectionNone
            scene.activeCamera.cameraPreset: Camera3D.CameraPresetIsometricLeft
            axisX.min: 0.0
            axisX.max: 100.0
            axisY.min: -5.0
            axisY.max: 5.0
            axisZ.min: 0.0
            axisZ.max: 100.0
            axisX.segmentCount: 10
            axisX.subSegmentCount: 2
            axisX.labelFormat: "%i"
            axisZ.segmentCount: 10
            axisZ.subSegmentCount: 2
            axisZ.labelFormat: "%i"
            axisY.segmentCount: 10
            axisY.subSegmentCount: 2
            axisY.labelFormat: "%i"
            axisY.title: "Z"
            axisX.title: "Latitude"
            axisZ.title: "Longitude"

            Surface3DSeries {
                id: surfaceSeries
                flatShadingEnabled: false
                drawMode: Surface3DSeries.DrawSurface

                ItemModelSurfaceDataProxy {
                    itemModel: automator.surfaceModel
                    rowRole: "y"
                    columnRole: "x"
                    yPosRole: "z"
                    /*rowRole: "longitude"
                    columnRole: "latitude"
                    yPosRole: "height"*/
                }
            }
        }
    }

}