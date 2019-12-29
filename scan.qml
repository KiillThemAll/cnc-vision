import QtQuick 2.10
import QtQuick.Window 2.10
import QtQuick.Controls 2.6
import QtQuick.Layouts 1.12
import QtDataVisualization 1.14

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

        Text {
            font.pointSize: 20
            text: automator.message
        }

        Button {
            text: "Approve scan"

            onClicked: automator.scanApproved = true
        }

        Button {
            text: "Clear scan"

            onClicked: automator.surfaceModel.removeAll()
        }


    }

    Item {
        id: surfaceView
        width: parent.width

        anchors.top: controls.bottom
        anchors.left: parent.left

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
                colorStyle: Theme3D.ColorStyleRangeGradient
                baseGradients: [surfaceGradient]
            }
            shadowQuality: AbstractGraph3D.ShadowQualityMedium
            selectionMode: AbstractGraph3D.SelectionSlice | AbstractGraph3D.SelectionItemAndRow
            scene.activeCamera.cameraPreset: Camera3D.CameraPresetIsometricLeft
            axisX.min: 0.0
            axisX.max: 1500.0
            axisY.min: 0.0
            axisY.max: 1500.0
            axisZ.min: -5
            axisZ.max: 5
            axisX.segmentCount: 10
            axisX.subSegmentCount: 2
            axisX.labelFormat: "%i"
            axisZ.segmentCount: 10
            axisZ.subSegmentCount: 2
            axisZ.labelFormat: "%i"
            axisY.segmentCount: 10
            axisY.subSegmentCount: 2
            axisY.labelFormat: "%i"
            axisY.title: "Height"
            axisX.title: "Latitude"
            axisZ.title: "Longitude"

            Surface3DSeries {
                id: surfaceSeries
                flatShadingEnabled: false
                drawMode: Surface3DSeries.DrawSurface

                ItemModelSurfaceDataProxy {
                    itemModel: automator.surfaceModel
                    rowRole: "x"
                    columnRole: "y"
                    yPosRole: "z"
                }
            }
        }
    }

}