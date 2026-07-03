import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtGraphicalEffects 1.15

Rectangle {
    id: root
    // 初始尺寸，实际大小跟随外部窗口变化
    width: 800
    height: 500
    color: "#f5f5f5"
    // 聊天气泡统一配色（左侧灰底，文字白色）
    property color leftbubbleColor: "#ababab"   // 浅灰色背景
    property color rightbubbleColor: "#4da3ff"   


    // 信号定义 - 与 Python 回调对接
    signal manualButtonPressed()
    signal manualButtonReleased()
    signal autoButtonClicked()
    signal abortButtonClicked()
    signal modeButtonClicked()
    signal sendButtonClicked(string text)
    signal settingsButtonClicked()
    // 标题栏相关信号
    signal titleMinimize()
    signal titleClose()
    signal titleDragStart(real mouseX, real mouseY)
    signal titleDragMoveTo(real mouseX, real mouseY)
    signal titleDragEnd()

    // 主布局
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 0
        spacing: 0

        // 自定义标题栏：最小化、关闭、可拖动
        Rectangle {
            id: titleBar
            Layout.fillWidth: true
            Layout.preferredHeight: 36
            color: "#f7f8fa"
            border.width: 0
            // contentItem: Text { text: modeBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 24; color: "#1d2129"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            // 整条标题栏拖动（使用屏幕坐标，避免累计误差导致抖动）
            // 放在最底层，让按钮的 MouseArea 可以优先响应
            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton
                onPressed: {
                    root.titleDragStart(mouse.x, mouse.y)
                }
                onPositionChanged: {
                    if (pressed) {
                        root.titleDragMoveTo(mouse.x, mouse.y)
                    }
                }
                onReleased: {
                    root.titleDragEnd()
                }
                z: 0  // 最底层
            }

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 10
                anchors.rightMargin: 8
                spacing: 8
                z: 1  // 按钮层在拖动层上方

                // 左侧拖动区域
                Item { id: dragArea; Layout.fillWidth: true; Layout.fillHeight: true }

                // 最小化
                Rectangle {
                    id: btnMin
                    width: 24; height: 24; radius: 6
                    color: btnMinMouse.pressed ? "#e5e6eb" : (btnMinMouse.containsMouse ? "#f2f3f5" : "transparent")
                    z: 2  // 确保按钮在最上层
                    Text { anchors.centerIn: parent; text: "–"; font.pixelSize: 14; color: "#4e5969" }
                    MouseArea {
                        id: btnMinMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: root.titleMinimize()
                    }
                }

                // 关闭
                Rectangle {
                    id: btnClose
                    width: 24; height: 24; radius: 6
                    color: btnCloseMouse.pressed ? "#f53f3f" : (btnCloseMouse.containsMouse ? "#ff7875" : "transparent")
                    z: 2  // 确保按钮在最上层
                    Text { anchors.centerIn: parent; text: "×"; font.pixelSize: 14; color: btnCloseMouse.containsMouse ? "white" : "#86909c" }
                    MouseArea {
                        id: btnCloseMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: root.titleClose()
                    }
                }
            }
        }

        // 状态卡片区域
        Rectangle {
            id: statusCard
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "transparent"

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 20

                // 状态标签
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 48
                    color: "#E3F2FD"
                    radius: 12
                    //contentItem: Text { text: settingsBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 24; color: "#1d2129"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    Text {
                        anchors.centerIn: parent
                        text: displayModel ? displayModel.statusText : "状态: 未连接"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 24
                        font.weight: Font.Bold
                        color: "#2196F3"
                    }
                }

                // 聊天区域：从下往上更新，仿即时聊天窗口
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 160
                    Layout.fillHeight: true
                    color: "transparent"

                    ListView {
                        id: chatList
                        anchors.fill: parent
                        anchors.margins: 8
                        clip: true
                        // 减小行间距，让 llmcmd 结果与下一条气泡更紧凑
                        spacing: 4
                        model: displayModel ? displayModel.chatMessages : []
                        boundsBehavior: Flickable.StopAtBounds
                        flickDeceleration: 2000

                        delegate: Item {
                            width: chatList.width
                            // 固定气泡宽度，高度随内容自适应
                            property string role: (modelData && modelData.role) ? modelData.role : ""
                            property string msgType: (modelData && modelData.type) ? modelData.type : "chat"
                            property string messageText: (modelData && modelData.text) ? modelData.text : ""

                            implicitHeight: msgType.indexOf("llmcmd") === 0
                                             ? llmcmdBox.implicitHeight + 4
                                             : bubble.implicitHeight + 4

                            // 普通对话消息：使用左右气泡
                            Rectangle {
                                id: bubble
                                // 仅在普通对话消息时显示气泡
                                visible: msgType === "chat"
                                // 圆角更大
                                radius: 18
                                
                                // 气泡根据文本大小自适应，但有最大宽度
                                width: bubbleText.width + 28
                                height: bubbleText.height + 28
                                
                                // 左侧/右侧气泡使用不同底色
                                color: role === "user" ? root.rightbubbleColor : root.leftbubbleColor
                                border.width: 0
                                border.color: "transparent"

                                // 用于外部 Item 计算高度
                                implicitHeight: height

                                anchors.top: parent.top
                                // 外边距增大，增加气泡周围空白
                                anchors.margins: 12
                                anchors.left: role === "user" ? undefined : parent.left
                                anchors.right: role === "user" ? parent.right : undefined

                                Text {
                                    id: bubbleText
                                    anchors.centerIn: parent
                                    
                                    text: messageText
                                    wrapMode: Text.Wrap
                                    font.family: "PingFang SC, Microsoft YaHei UI"
                                    // 字号调大并加粗
                                    font.pixelSize: 27
                                    font.weight: Font.Bold
                                    color: "white"

                                    // 限制最大宽度，超过则换行
                                    property real maxTextWidth: chatList.width * 0.8 - 28
                                    width: Math.min(implicitWidth, maxTextWidth)
                                }
                            }

                            // 工具调用结果：无气泡，直接文本输出
                            Rectangle {
                                id: llmcmdBox
                                // llmcmd 系列：普通/ok/error
                                visible: msgType.indexOf("llmcmd") === 0
                                color: "transparent"
                                radius: 0
                                // 宽度与左侧气泡保持一致
                                width: parent.width * 0.6
                                anchors.left: parent.left
                                anchors.margins: 12
                                implicitHeight: llmcmdText.implicitHeight + 8

                                Text {
                                    id: llmcmdText
                                    anchors.fill: parent
                                    anchors.margins: 4
                                    text: messageText
                                    wrapMode: Text.WordWrap
                                    font.family: "PingFang SC, Microsoft YaHei UI"
                                    font.pixelSize: 24
                                    // 普通为灰色，ok 绿色，error 红色
                                    color: msgType === "llmcmd_ok" ? "#52c41a"
                                          : (msgType === "llmcmd_error" ? "#ff4d4f" : "#86909c")
                                }
                            }
                        }

                        // 底部预留额外空白，避免最后一条被底部区域遮挡
                        footer: Item { height: 24 }

                        // 新消息到来时始终滚动到底部，保证“最新在下方”
                        onCountChanged: {
                            positionViewAtEnd()
                        }
                        onModelChanged: {
                            positionViewAtEnd()
                        }
                        Component.onCompleted: {
                            positionViewAtEnd()
                        }
                    }
                }
            }
        }

        // 按钮区域（统一配色与尺寸）
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 72
            color: "#f7f8fa"

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 16
                anchors.rightMargin: 16
                anchors.bottomMargin: 12
                spacing: 10

                // 手动模式按钮（按住说话） - 主色
                Button {
                    id: manualBtn
                    Layout.preferredWidth: 140
                    Layout.preferredHeight: 40
                    text: "按住后说话"
                    visible: displayModel ? !displayModel.autoMode : true

                    background: Rectangle {
                        color: manualBtn.pressed ? "#0e42d2" : (manualBtn.hovered ? "#4080ff" : "#165dff")
                        radius: 8

                        Behavior on color { ColorAnimation { duration: 120; easing.type: Easing.OutCubic } }
                    }

                    contentItem: Text {
                        text: manualBtn.text
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 24
                        color: "white"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    onPressed: { manualBtn.text = "松开以停止"; root.manualButtonPressed() }
                    onReleased: { manualBtn.text = "按住后说话"; root.manualButtonReleased() }
                }

                // 自动模式按钮 - 主色
                Button {
                    id: autoBtn
                    Layout.preferredWidth: 140
                    Layout.preferredHeight: 40
                    text: displayModel ? displayModel.buttonText : "开始对话"
                    visible: displayModel ? displayModel.autoMode : false

                    background: Rectangle {
                        color: autoBtn.pressed ? "#0e42d2" : (autoBtn.hovered ? "#4080ff" : "#165dff")
                        radius: 8
                        Behavior on color { ColorAnimation { duration: 120; easing.type: Easing.OutCubic } }
                    }

                    contentItem: Text { text: autoBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 24; color: "white"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    onClicked: root.autoButtonClicked()
                }

                // 打断对话 - 次要色
                Button {
                    id: abortBtn
                    Layout.preferredWidth: 120
                    Layout.preferredHeight: 40
                    text: "打断对话"

                    background: Rectangle { color: abortBtn.pressed ? "#e5e6eb" : (abortBtn.hovered ? "#f2f3f5" : "#eceff3"); radius: 8 }
                    contentItem: Text { text: abortBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 24; color: "#1d2129"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    onClicked: root.abortButtonClicked()
                }

                // 输入 + 发送
                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    spacing: 8

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 40
                        color: "white"
                        radius: 8
                        border.color: textInput.activeFocus ? "#165dff" : "#e5e6eb"
                        border.width: 1

                        TextInput {
                            id: textInput
                            anchors.fill: parent
                            anchors.leftMargin: 12
                            anchors.rightMargin: 12
                            verticalAlignment: TextInput.AlignVCenter
                            font.family: "PingFang SC, Microsoft YaHei UI"
                            font.pixelSize: 24
                            color: "#333333"
                            selectByMouse: true
                            clip: true

                            // 占位符
                            Text { anchors.fill: parent; text: "输入文字..."; font: textInput.font; color: "#c9cdd4"; verticalAlignment: Text.AlignVCenter; visible: !textInput.text && !textInput.activeFocus }

                            Keys.onReturnPressed: { if (textInput.text.trim().length > 0) { root.sendButtonClicked(textInput.text); textInput.text = "" } }
                        }
                    }

                    Button {
                        id: sendBtn
                        Layout.preferredWidth: 84
                        Layout.preferredHeight: 40
                        text: "发送"
                        background: Rectangle { color: sendBtn.pressed ? "#0e42d2" : (sendBtn.hovered ? "#4080ff" : "#165dff"); radius: 8 }
                        contentItem: Text { text: sendBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 24; color: "white"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: { if (textInput.text.trim().length > 0) { root.sendButtonClicked(textInput.text); textInput.text = "" } }
                    }
                }

                // 模式（次要）
                Button {
                    id: modeBtn
                    Layout.preferredWidth: 120
                    Layout.preferredHeight: 40
                    text: displayModel ? displayModel.modeText : "手动对话"
                    background: Rectangle { color: modeBtn.pressed ? "#e5e6eb" : (modeBtn.hovered ? "#f2f3f5" : "#eceff3"); radius: 8 }
                    contentItem: Text { text: modeBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "#1d2129"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    onClicked: root.modeButtonClicked()
                }

                // 设置（次要）
                Button {
                    id: settingsBtn
                    Layout.preferredWidth: 120
                    Layout.preferredHeight: 40
                    text: "参数配置"
                    background: Rectangle { color: settingsBtn.pressed ? "#e5e6eb" : (settingsBtn.hovered ? "#f2f3f5" : "#eceff3"); radius: 8 }
                    contentItem: Text { text: settingsBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "#1d2129"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    onClicked: root.settingsButtonClicked()
                }
            }
        }
    }
}
