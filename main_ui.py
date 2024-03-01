# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.15.10
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1250, 800)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMaximumSize(QtCore.QSize(1250, 800))
        MainWindow.setStyleSheet("QLineEdit {\n"
"    padding: 3px\n"
"}\n"
"\n"
"#serverConfig QLineEdit {\n"
"    margin-left: 5px;\n"
"    padding: 3px\n"
"}\n"
"\n"
"#outputFrame QScrollArea {background:blue}")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_11 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.topFrame = QtWidgets.QFrame(self.centralwidget)
        self.topFrame.setMaximumSize(QtCore.QSize(16777215, 80))
        self.topFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.topFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.topFrame.setObjectName("topFrame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.topFrame)
        self.horizontalLayout.setContentsMargins(-1, -1, 17, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_7 = QtWidgets.QFrame(self.topFrame)
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame_7)
        self.verticalLayout.setContentsMargins(-1, 0, -1, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.appTitle = QtWidgets.QLabel(self.frame_7)
        self.appTitle.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.appTitle.setFont(font)
        self.appTitle.setObjectName("appTitle")
        self.verticalLayout.addWidget(self.appTitle)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.developer = QtWidgets.QLabel(self.frame_7)
        self.developer.setMaximumSize(QtCore.QSize(90, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.developer.setFont(font)
        self.developer.setObjectName("developer")
        self.horizontalLayout_3.addWidget(self.developer)
        self.github = QtWidgets.QLabel(self.frame_7)
        self.github.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setUnderline(False)
        font.setWeight(50)
        self.github.setFont(font)
        self.github.setOpenExternalLinks(True)
        self.github.setObjectName("github")
        self.horizontalLayout_3.addWidget(self.github)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout.addWidget(self.frame_7)
        spacerItem = QtWidgets.QSpacerItem(87, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.adapterConfig = QtWidgets.QFrame(self.topFrame)
        self.adapterConfig.setMinimumSize(QtCore.QSize(337, 0))
        self.adapterConfig.setMaximumSize(QtCore.QSize(337, 16777215))
        self.adapterConfig.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.adapterConfig.setFrameShadow(QtWidgets.QFrame.Raised)
        self.adapterConfig.setObjectName("adapterConfig")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.adapterConfig)
        self.horizontalLayout_4.setSpacing(11)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.adapterLabel = QtWidgets.QLabel(self.adapterConfig)
        self.adapterLabel.setMinimumSize(QtCore.QSize(51, 0))
        self.adapterLabel.setMaximumSize(QtCore.QSize(51, 16777215))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.adapterLabel.setFont(font)
        self.adapterLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.adapterLabel.setObjectName("adapterLabel")
        self.horizontalLayout_4.addWidget(self.adapterLabel)
        self.adapterSelect = QtWidgets.QComboBox(self.adapterConfig)
        self.adapterSelect.setMinimumSize(QtCore.QSize(0, 28))
        self.adapterSelect.setMaximumSize(QtCore.QSize(16777215, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.adapterSelect.setFont(font)
        self.adapterSelect.setObjectName("adapterSelect")
        self.horizontalLayout_4.addWidget(self.adapterSelect)
        self.horizontalLayout.addWidget(self.adapterConfig)
        self.serverConfig = QtWidgets.QFrame(self.topFrame)
        self.serverConfig.setMinimumSize(QtCore.QSize(337, 0))
        self.serverConfig.setMaximumSize(QtCore.QSize(337, 16777215))
        self.serverConfig.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.serverConfig.setFrameShadow(QtWidgets.QFrame.Raised)
        self.serverConfig.setObjectName("serverConfig")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.serverConfig)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.serverLabel = QtWidgets.QLabel(self.serverConfig)
        self.serverLabel.setMinimumSize(QtCore.QSize(42, 0))
        self.serverLabel.setMaximumSize(QtCore.QSize(42, 16777215))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.serverLabel.setFont(font)
        self.serverLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.serverLabel.setObjectName("serverLabel")
        self.horizontalLayout_2.addWidget(self.serverLabel)
        self.serverInput = QtWidgets.QLineEdit(self.serverConfig)
        self.serverInput.setMinimumSize(QtCore.QSize(255, 0))
        self.serverInput.setMaximumSize(QtCore.QSize(305, 16777215))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.serverInput.setFont(font)
        self.serverInput.setObjectName("serverInput")
        self.horizontalLayout_2.addWidget(self.serverInput)
        self.horizontalLayout.addWidget(self.serverConfig)
        self.verticalLayout_11.addWidget(self.topFrame)
        self.midFrame = QtWidgets.QFrame(self.centralwidget)
        self.midFrame.setMinimumSize(QtCore.QSize(0, 390))
        self.midFrame.setMaximumSize(QtCore.QSize(16777215, 390))
        self.midFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.midFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.midFrame.setObjectName("midFrame")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.midFrame)
        self.verticalLayout_9.setContentsMargins(-1, 0, -1, -1)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.connectionConfig = QtWidgets.QFrame(self.midFrame)
        self.connectionConfig.setMinimumSize(QtCore.QSize(0, 50))
        self.connectionConfig.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.connectionConfig.setFont(font)
        self.connectionConfig.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.connectionConfig.setFrameShadow(QtWidgets.QFrame.Raised)
        self.connectionConfig.setObjectName("connectionConfig")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.connectionConfig)
        self.horizontalLayout_5.setContentsMargins(-1, 11, 17, 11)
        self.horizontalLayout_5.setSpacing(15)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.connectionStatus = QtWidgets.QCheckBox(self.connectionConfig)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.connectionStatus.setFont(font)
        self.connectionStatus.setObjectName("connectionStatus")
        self.horizontalLayout_5.addWidget(self.connectionStatus)
        spacerItem1 = QtWidgets.QSpacerItem(782, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem1)
        self.connectBtn = QtWidgets.QPushButton(self.connectionConfig)
        self.connectBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.connectBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.connectBtn.setFont(font)
        self.connectBtn.setObjectName("connectBtn")
        self.horizontalLayout_5.addWidget(self.connectBtn)
        self.disconnectBtn = QtWidgets.QPushButton(self.connectionConfig)
        self.disconnectBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.disconnectBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.disconnectBtn.setFont(font)
        self.disconnectBtn.setObjectName("disconnectBtn")
        self.horizontalLayout_5.addWidget(self.disconnectBtn)
        self.verticalLayout_9.addWidget(self.connectionConfig)
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setContentsMargins(0, -1, -1, -1)
        self.horizontalLayout_14.setSpacing(0)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.controlConfig = QtWidgets.QFrame(self.midFrame)
        self.controlConfig.setMinimumSize(QtCore.QSize(531, 330))
        self.controlConfig.setMaximumSize(QtCore.QSize(531, 330))
        self.controlConfig.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.controlConfig.setFrameShadow(QtWidgets.QFrame.Raised)
        self.controlConfig.setObjectName("controlConfig")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.controlConfig)
        self.verticalLayout_3.setContentsMargins(-1, 0, -1, -1)
        self.verticalLayout_3.setSpacing(5)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.frame_6 = QtWidgets.QFrame(self.controlConfig)
        self.frame_6.setMinimumSize(QtCore.QSize(160, 0))
        self.frame_6.setMaximumSize(QtCore.QSize(170, 16777215))
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_6)
        self.verticalLayout_4.setSpacing(15)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.powerOnBtn = QtWidgets.QPushButton(self.frame_6)
        self.powerOnBtn.setMinimumSize(QtCore.QSize(0, 32))
        self.powerOnBtn.setMaximumSize(QtCore.QSize(16777215, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.powerOnBtn.setFont(font)
        self.powerOnBtn.setObjectName("powerOnBtn")
        self.verticalLayout_4.addWidget(self.powerOnBtn)
        self.powerOffBtn = QtWidgets.QPushButton(self.frame_6)
        self.powerOffBtn.setMinimumSize(QtCore.QSize(0, 32))
        self.powerOffBtn.setMaximumSize(QtCore.QSize(16777215, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.powerOffBtn.setFont(font)
        self.powerOffBtn.setObjectName("powerOffBtn")
        self.verticalLayout_4.addWidget(self.powerOffBtn)
        self.brakeReleaseBtn = QtWidgets.QPushButton(self.frame_6)
        self.brakeReleaseBtn.setMinimumSize(QtCore.QSize(0, 32))
        self.brakeReleaseBtn.setMaximumSize(QtCore.QSize(16777215, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.brakeReleaseBtn.setFont(font)
        self.brakeReleaseBtn.setObjectName("brakeReleaseBtn")
        self.verticalLayout_4.addWidget(self.brakeReleaseBtn)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem2)
        self.horizontalLayout_7.addWidget(self.frame_6)
        spacerItem3 = QtWidgets.QSpacerItem(77, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_7.addItem(spacerItem3)
        self.frame_9 = QtWidgets.QFrame(self.controlConfig)
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_9)
        self.verticalLayout_5.setSpacing(15)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setSpacing(15)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.startFreeDriveBtn = QtWidgets.QPushButton(self.frame_9)
        self.startFreeDriveBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.startFreeDriveBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.startFreeDriveBtn.setFont(font)
        self.startFreeDriveBtn.setObjectName("startFreeDriveBtn")
        self.horizontalLayout_9.addWidget(self.startFreeDriveBtn)
        self.endFreeDriveBtn = QtWidgets.QPushButton(self.frame_9)
        self.endFreeDriveBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.endFreeDriveBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.endFreeDriveBtn.setFont(font)
        self.endFreeDriveBtn.setObjectName("endFreeDriveBtn")
        self.horizontalLayout_9.addWidget(self.endFreeDriveBtn)
        self.verticalLayout_5.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setContentsMargins(-1, 10, -1, -1)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.speedLabel = QtWidgets.QLabel(self.frame_9)
        self.speedLabel.setMaximumSize(QtCore.QSize(16777215, 40))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.speedLabel.setFont(font)
        self.speedLabel.setObjectName("speedLabel")
        self.horizontalLayout_8.addWidget(self.speedLabel)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem4)
        self.verticalLayout_5.addLayout(self.horizontalLayout_8)
        self.speedControl = QtWidgets.QSlider(self.frame_9)
        self.speedControl.setOrientation(QtCore.Qt.Horizontal)
        self.speedControl.setObjectName("speedControl")
        self.verticalLayout_5.addWidget(self.speedControl)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem5)
        self.horizontalLayout_7.addWidget(self.frame_9)
        self.verticalLayout_3.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setContentsMargins(15, -1, -1, 30)
        self.horizontalLayout_6.setSpacing(10)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.urpFileInput = QtWidgets.QLineEdit(self.controlConfig)
        self.urpFileInput.setMinimumSize(QtCore.QSize(340, 28))
        self.urpFileInput.setMaximumSize(QtCore.QSize(340, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.urpFileInput.setFont(font)
        self.urpFileInput.setObjectName("urpFileInput")
        self.horizontalLayout_6.addWidget(self.urpFileInput)
        self.loadUrpBtn = QtWidgets.QPushButton(self.controlConfig)
        self.loadUrpBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.loadUrpBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.loadUrpBtn.setFont(font)
        self.loadUrpBtn.setObjectName("loadUrpBtn")
        self.horizontalLayout_6.addWidget(self.loadUrpBtn)
        self.verticalLayout_3.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setSpacing(15)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setContentsMargins(12, -1, -1, -1)
        self.verticalLayout_6.setSpacing(15)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.playBtn = QtWidgets.QPushButton(self.controlConfig)
        self.playBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.playBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.playBtn.setFont(font)
        self.playBtn.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.playBtn.setObjectName("playBtn")
        self.verticalLayout_6.addWidget(self.playBtn)
        self.stopBtn = QtWidgets.QPushButton(self.controlConfig)
        self.stopBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.stopBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.stopBtn.setFont(font)
        self.stopBtn.setObjectName("stopBtn")
        self.verticalLayout_6.addWidget(self.stopBtn)
        self.horizontalLayout_10.addLayout(self.verticalLayout_6)
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setSpacing(15)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.pauseBtn = QtWidgets.QPushButton(self.controlConfig)
        self.pauseBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.pauseBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pauseBtn.setFont(font)
        self.pauseBtn.setObjectName("pauseBtn")
        self.verticalLayout_7.addWidget(self.pauseBtn)
        self.shutDownBtn = QtWidgets.QPushButton(self.controlConfig)
        self.shutDownBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.shutDownBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.shutDownBtn.setFont(font)
        self.shutDownBtn.setObjectName("shutDownBtn")
        self.verticalLayout_7.addWidget(self.shutDownBtn)
        self.horizontalLayout_10.addLayout(self.verticalLayout_7)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setSpacing(15)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.unlockBtn = QtWidgets.QPushButton(self.controlConfig)
        self.unlockBtn.setMinimumSize(QtCore.QSize(185, 32))
        self.unlockBtn.setMaximumSize(QtCore.QSize(185, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.unlockBtn.setFont(font)
        self.unlockBtn.setObjectName("unlockBtn")
        self.verticalLayout_8.addWidget(self.unlockBtn)
        self.closePopUpBtn = QtWidgets.QPushButton(self.controlConfig)
        self.closePopUpBtn.setMinimumSize(QtCore.QSize(185, 32))
        self.closePopUpBtn.setMaximumSize(QtCore.QSize(185, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.closePopUpBtn.setFont(font)
        self.closePopUpBtn.setObjectName("closePopUpBtn")
        self.verticalLayout_8.addWidget(self.closePopUpBtn)
        self.horizontalLayout_10.addLayout(self.verticalLayout_8)
        self.verticalLayout_3.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_14.addWidget(self.controlConfig)
        self.motionConfig = QtWidgets.QFrame(self.midFrame)
        self.motionConfig.setMinimumSize(QtCore.QSize(661, 330))
        self.motionConfig.setMaximumSize(QtCore.QSize(661, 330))
        self.motionConfig.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.motionConfig.setFrameShadow(QtWidgets.QFrame.Raised)
        self.motionConfig.setObjectName("motionConfig")
        self.verticalLayout_10 = QtWidgets.QVBoxLayout(self.motionConfig)
        self.verticalLayout_10.setContentsMargins(20, 15, 15, -1)
        self.verticalLayout_10.setSpacing(15)
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setSpacing(7)
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.refreshBtn = QtWidgets.QPushButton(self.motionConfig)
        self.refreshBtn.setMinimumSize(QtCore.QSize(32, 32))
        self.refreshBtn.setMaximumSize(QtCore.QSize(32, 32))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.refreshBtn.setFont(font)
        self.refreshBtn.setObjectName("refreshBtn")
        self.horizontalLayout_11.addWidget(self.refreshBtn)
        self.motionSelect = QtWidgets.QComboBox(self.motionConfig)
        self.motionSelect.setMinimumSize(QtCore.QSize(290, 28))
        self.motionSelect.setMaximumSize(QtCore.QSize(290, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.motionSelect.setFont(font)
        self.motionSelect.setObjectName("motionSelect")
        self.horizontalLayout_11.addWidget(self.motionSelect)
        self.loadMotionBtn = QtWidgets.QPushButton(self.motionConfig)
        self.loadMotionBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.loadMotionBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.loadMotionBtn.setFont(font)
        self.loadMotionBtn.setObjectName("loadMotionBtn")
        self.horizontalLayout_11.addWidget(self.loadMotionBtn)
        self.deleteBtn = QtWidgets.QPushButton(self.motionConfig)
        self.deleteBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.deleteBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.deleteBtn.setFont(font)
        self.deleteBtn.setObjectName("deleteBtn")
        self.horizontalLayout_11.addWidget(self.deleteBtn)
        self.verticalLayout_10.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setContentsMargins(-1, -1, -1, 5)
        self.horizontalLayout_12.setSpacing(7)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.motionNameInput = QtWidgets.QLineEdit(self.motionConfig)
        self.motionNameInput.setMinimumSize(QtCore.QSize(280, 28))
        self.motionNameInput.setMaximumSize(QtCore.QSize(280, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.motionNameInput.setFont(font)
        self.motionNameInput.setObjectName("motionNameInput")
        self.horizontalLayout_12.addWidget(self.motionNameInput)
        self.recordBtn = QtWidgets.QPushButton(self.motionConfig)
        self.recordBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.recordBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.recordBtn.setFont(font)
        self.recordBtn.setObjectName("recordBtn")
        self.horizontalLayout_12.addWidget(self.recordBtn)
        self.runRobotBtn = QtWidgets.QPushButton(self.motionConfig)
        self.runRobotBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.runRobotBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.runRobotBtn.setFont(font)
        self.runRobotBtn.setObjectName("runRobotBtn")
        self.horizontalLayout_12.addWidget(self.runRobotBtn)
        self.numRepetition = QtWidgets.QSpinBox(self.motionConfig)
        self.numRepetition.setMinimumSize(QtCore.QSize(0, 28))
        self.numRepetition.setMaximumSize(QtCore.QSize(16777215, 28))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.numRepetition.setFont(font)
        self.numRepetition.setObjectName("numRepetition")
        self.horizontalLayout_12.addWidget(self.numRepetition)
        self.verticalLayout_10.addLayout(self.horizontalLayout_12)
        self.motionTable = QtWidgets.QTableWidget(self.motionConfig)
        self.motionTable.setMinimumSize(QtCore.QSize(620, 0))
        self.motionTable.setMaximumSize(QtCore.QSize(620, 16777215))
        self.motionTable.setAlternatingRowColors(False)
        self.motionTable.setShowGrid(True)
        self.motionTable.setWordWrap(False)
        self.motionTable.setRowCount(10)
        self.motionTable.setObjectName("motionTable")
        self.motionTable.setColumnCount(6)
        item = QtWidgets.QTableWidgetItem()
        self.motionTable.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.motionTable.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.motionTable.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.motionTable.setHorizontalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        self.motionTable.setHorizontalHeaderItem(4, item)
        item = QtWidgets.QTableWidgetItem()
        self.motionTable.setHorizontalHeaderItem(5, item)
        self.motionTable.horizontalHeader().setCascadingSectionResizes(False)
        self.motionTable.horizontalHeader().setDefaultSectionSize(95)
        self.motionTable.horizontalHeader().setMinimumSectionSize(95)
        self.motionTable.horizontalHeader().setSortIndicatorShown(False)
        self.motionTable.verticalHeader().setDefaultSectionSize(25)
        self.motionTable.verticalHeader().setMinimumSectionSize(25)
        self.verticalLayout_10.addWidget(self.motionTable)
        self.horizontalLayout_14.addWidget(self.motionConfig)
        self.verticalLayout_9.addLayout(self.horizontalLayout_14)
        self.verticalLayout_11.addWidget(self.midFrame)
        self.outputFrame = QtWidgets.QFrame(self.centralwidget)
        self.outputFrame.setMaximumSize(QtCore.QSize(16777215, 350))
        self.outputFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.outputFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.outputFrame.setObjectName("outputFrame")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.outputFrame)
        self.verticalLayout_2.setContentsMargins(25, 0, 16, -1)
        self.verticalLayout_2.setSpacing(7)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_4 = QtWidgets.QFrame(self.outputFrame)
        self.frame_4.setMinimumSize(QtCore.QSize(0, 40))
        self.frame_4.setMaximumSize(QtCore.QSize(16777215, 40))
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.frame_4)
        self.horizontalLayout_13.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_13.setSpacing(15)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.outputLabel = QtWidgets.QLabel(self.frame_4)
        self.outputLabel.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.outputLabel.setFont(font)
        self.outputLabel.setObjectName("outputLabel")
        self.horizontalLayout_13.addWidget(self.outputLabel)
        spacerItem6 = QtWidgets.QSpacerItem(909, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_13.addItem(spacerItem6)
        self.clearOutputBtn = QtWidgets.QPushButton(self.frame_4)
        self.clearOutputBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.clearOutputBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.clearOutputBtn.setFont(font)
        self.clearOutputBtn.setObjectName("clearOutputBtn")
        self.horizontalLayout_13.addWidget(self.clearOutputBtn)
        self.clearTableBtn = QtWidgets.QPushButton(self.frame_4)
        self.clearTableBtn.setMinimumSize(QtCore.QSize(136, 32))
        self.clearTableBtn.setMaximumSize(QtCore.QSize(136, 32))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.clearTableBtn.setFont(font)
        self.clearTableBtn.setObjectName("clearTableBtn")
        self.horizontalLayout_13.addWidget(self.clearTableBtn)
        self.verticalLayout_2.addWidget(self.frame_4)
        self.outputResponse = QtWidgets.QTextEdit(self.outputFrame)
        self.outputResponse.setMinimumSize(QtCore.QSize(1175, 190))
        self.outputResponse.setMaximumSize(QtCore.QSize(1175, 190))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.outputResponse.setFont(font)
        self.outputResponse.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.outputResponse.setObjectName("outputResponse")
        self.verticalLayout_2.addWidget(self.outputResponse)
        spacerItem7 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem7)
        self.progressBar = QtWidgets.QProgressBar(self.outputFrame)
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.progressBar.setFont(font)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setInvertedAppearance(False)
        self.progressBar.setObjectName("progressBar")
        self.verticalLayout_2.addWidget(self.progressBar)
        spacerItem8 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem8)
        self.verticalLayout_11.addWidget(self.outputFrame)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.appTitle.setText(_translate("MainWindow", "Universal Robots Communication Tool"))
        self.developer.setText(_translate("MainWindow", "Developed by"))
        self.github.setText(_translate("MainWindow", "<html><head/><body><p>Yi Xian (<a href=\"https://github.com/yx-elite\"><span style=\" text-decoration: underline; color:#000000;\">GitHub</span></a>)</p></body></html>"))
        self.adapterLabel.setText(_translate("MainWindow", "Adapter"))
        self.serverLabel.setText(_translate("MainWindow", "Server"))
        self.connectionStatus.setText(_translate("MainWindow", "Connected"))
        self.connectBtn.setText(_translate("MainWindow", "Connect"))
        self.disconnectBtn.setText(_translate("MainWindow", "Disconnect"))
        self.powerOnBtn.setText(_translate("MainWindow", "Power On"))
        self.powerOffBtn.setText(_translate("MainWindow", "Power Off"))
        self.brakeReleaseBtn.setText(_translate("MainWindow", "Brake Release"))
        self.startFreeDriveBtn.setText(_translate("MainWindow", "Start Freedrive"))
        self.endFreeDriveBtn.setText(_translate("MainWindow", "End Freedrive"))
        self.speedLabel.setText(_translate("MainWindow", "Speed Slider"))
        self.loadUrpBtn.setText(_translate("MainWindow", "Load URP"))
        self.playBtn.setText(_translate("MainWindow", "Play"))
        self.stopBtn.setText(_translate("MainWindow", "Stop"))
        self.pauseBtn.setText(_translate("MainWindow", "Pause"))
        self.shutDownBtn.setText(_translate("MainWindow", "Shut Down"))
        self.unlockBtn.setText(_translate("MainWindow", "Unlock Protective Stop"))
        self.closePopUpBtn.setText(_translate("MainWindow", "Close Safety Popup"))
        self.refreshBtn.setText(_translate("MainWindow", "↻"))
        self.loadMotionBtn.setText(_translate("MainWindow", "Load Motion"))
        self.deleteBtn.setText(_translate("MainWindow", "Delete"))
        self.recordBtn.setText(_translate("MainWindow", "Record"))
        self.runRobotBtn.setText(_translate("MainWindow", "Run Robot"))
        self.motionTable.setSortingEnabled(False)
        item = self.motionTable.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "X"))
        item = self.motionTable.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Y"))
        item = self.motionTable.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Z"))
        item = self.motionTable.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "RX"))
        item = self.motionTable.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "RY"))
        item = self.motionTable.horizontalHeaderItem(5)
        item.setText(_translate("MainWindow", "RZ"))
        self.outputLabel.setText(_translate("MainWindow", "System Output"))
        self.clearOutputBtn.setText(_translate("MainWindow", "Clear Output"))
        self.clearTableBtn.setText(_translate("MainWindow", "Clear Table"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
