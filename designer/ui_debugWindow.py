# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\debugWindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1096, 889)
        Form.setMinimumSize(QtCore.QSize(860, 433))
        Form.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.gridLayout_12 = QtWidgets.QGridLayout(Form)
        self.gridLayout_12.setObjectName("gridLayout_12")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(Form)
        self.label.setMaximumSize(QtCore.QSize(223, 16777215))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label, 0, QtCore.Qt.AlignHCenter)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.tagCheckbox3 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox3.setMaximumSize(QtCore.QSize(108, 16777215))
        self.tagCheckbox3.setChecked(True)
        self.tagCheckbox3.setObjectName("tagCheckbox3")
        self.gridLayout_4.addWidget(self.tagCheckbox3, 2, 0, 1, 1)
        self.tagCheckbox5 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox5.setMaximumSize(QtCore.QSize(108, 16777215))
        self.tagCheckbox5.setChecked(True)
        self.tagCheckbox5.setObjectName("tagCheckbox5")
        self.gridLayout_4.addWidget(self.tagCheckbox5, 4, 0, 1, 1)
        self.tagCheckbox1 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox1.setMaximumSize(QtCore.QSize(108, 16777215))
        self.tagCheckbox1.setChecked(True)
        self.tagCheckbox1.setObjectName("tagCheckbox1")
        self.gridLayout_4.addWidget(self.tagCheckbox1, 0, 0, 1, 1)
        self.tagCheckbox2 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox2.setMaximumSize(QtCore.QSize(107, 16777215))
        self.tagCheckbox2.setChecked(True)
        self.tagCheckbox2.setTristate(False)
        self.tagCheckbox2.setObjectName("tagCheckbox2")
        self.gridLayout_4.addWidget(self.tagCheckbox2, 0, 1, 1, 1)
        self.tagCheckbox4 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox4.setMaximumSize(QtCore.QSize(107, 16777215))
        self.tagCheckbox4.setChecked(True)
        self.tagCheckbox4.setObjectName("tagCheckbox4")
        self.gridLayout_4.addWidget(self.tagCheckbox4, 2, 1, 1, 1)
        self.tagCheckbox6 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox6.setMaximumSize(QtCore.QSize(107, 16777215))
        self.tagCheckbox6.setChecked(True)
        self.tagCheckbox6.setObjectName("tagCheckbox6")
        self.gridLayout_4.addWidget(self.tagCheckbox6, 4, 1, 1, 1)
        self.tagCheckbox8 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox8.setMaximumSize(QtCore.QSize(107, 16777215))
        self.tagCheckbox8.setChecked(True)
        self.tagCheckbox8.setObjectName("tagCheckbox8")
        self.gridLayout_4.addWidget(self.tagCheckbox8, 5, 1, 1, 1)
        self.tagCheckbox7 = QtWidgets.QCheckBox(Form)
        self.tagCheckbox7.setMaximumSize(QtCore.QSize(108, 16777215))
        self.tagCheckbox7.setChecked(True)
        self.tagCheckbox7.setObjectName("tagCheckbox7")
        self.gridLayout_4.addWidget(self.tagCheckbox7, 5, 0, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_4)
        self.line = QtWidgets.QFrame(Form)
        self.line.setMaximumSize(QtCore.QSize(223, 16777215))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.pitchValue = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pitchValue.sizePolicy().hasHeightForWidth())
        self.pitchValue.setSizePolicy(sizePolicy)
        self.pitchValue.setMaximumSize(QtCore.QSize(133, 16777215))
        self.pitchValue.setReadOnly(True)
        self.pitchValue.setObjectName("pitchValue")
        self.gridLayout.addWidget(self.pitchValue, 2, 1, 1, 1)
        self.pitchLabel = QtWidgets.QLabel(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pitchLabel.sizePolicy().hasHeightForWidth())
        self.pitchLabel.setSizePolicy(sizePolicy)
        self.pitchLabel.setMaximumSize(QtCore.QSize(82, 16777215))
        self.pitchLabel.setObjectName("pitchLabel")
        self.gridLayout.addWidget(self.pitchLabel, 2, 0, 1, 1)
        self.yawLabel = QtWidgets.QLabel(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.yawLabel.sizePolicy().hasHeightForWidth())
        self.yawLabel.setSizePolicy(sizePolicy)
        self.yawLabel.setMaximumSize(QtCore.QSize(82, 16777215))
        self.yawLabel.setObjectName("yawLabel")
        self.gridLayout.addWidget(self.yawLabel, 1, 0, 1, 1)
        self.yawValue = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.yawValue.sizePolicy().hasHeightForWidth())
        self.yawValue.setSizePolicy(sizePolicy)
        self.yawValue.setMaximumSize(QtCore.QSize(133, 16777215))
        self.yawValue.setObjectName("yawValue")
        self.gridLayout.addWidget(self.yawValue, 1, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.resetGyroBtn = QtWidgets.QPushButton(Form)
        self.resetGyroBtn.setMaximumSize(QtCore.QSize(223, 16777215))
        self.resetGyroBtn.setObjectName("resetGyroBtn")
        self.verticalLayout.addWidget(self.resetGyroBtn, 0, QtCore.Qt.AlignHCenter)
        self.line_2 = QtWidgets.QFrame(Form)
        self.line_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.verticalLayout.addWidget(self.line_2, 0, QtCore.Qt.AlignHCenter)
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setMaximumSize(QtCore.QSize(70, 16777215))
        self.label_3.setObjectName("label_3")
        self.gridLayout_3.addWidget(self.label_3, 0, 0, 1, 1)
        self.detectedTagsValue = QtWidgets.QLineEdit(Form)
        self.detectedTagsValue.setMaximumSize(QtCore.QSize(145, 16777215))
        self.detectedTagsValue.setObjectName("detectedTagsValue")
        self.gridLayout_3.addWidget(self.detectedTagsValue, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_3)
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setMaximumSize(QtCore.QSize(223, 16777215))
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4, 0, QtCore.Qt.AlignHCenter)
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_6 = QtWidgets.QLabel(Form)
        self.label_6.setMaximumSize(QtCore.QSize(54, 16777215))
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.gridLayout_6.addWidget(self.label_6, 0, 1, 1, 1)
        self.avgDepthYValue = QtWidgets.QLineEdit(Form)
        self.avgDepthYValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.avgDepthYValue.setObjectName("avgDepthYValue")
        self.gridLayout_6.addWidget(self.avgDepthYValue, 1, 2, 1, 1)
        self.avgDepthZValue = QtWidgets.QLineEdit(Form)
        self.avgDepthZValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.avgDepthZValue.setObjectName("avgDepthZValue")
        self.gridLayout_6.addWidget(self.avgDepthZValue, 1, 3, 1, 1)
        self.stdDepthYValue = QtWidgets.QLineEdit(Form)
        self.stdDepthYValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.stdDepthYValue.setObjectName("stdDepthYValue")
        self.gridLayout_6.addWidget(self.stdDepthYValue, 2, 2, 1, 1)
        self.stdDepthZValue = QtWidgets.QLineEdit(Form)
        self.stdDepthZValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.stdDepthZValue.setObjectName("stdDepthZValue")
        self.gridLayout_6.addWidget(self.stdDepthZValue, 2, 3, 1, 1)
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setMaximumSize(QtCore.QSize(41, 16777215))
        self.label_5.setObjectName("label_5")
        self.gridLayout_6.addWidget(self.label_5, 1, 0, 1, 1)
        self.stdDepthXValue = QtWidgets.QLineEdit(Form)
        self.stdDepthXValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.stdDepthXValue.setObjectName("stdDepthXValue")
        self.gridLayout_6.addWidget(self.stdDepthXValue, 2, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(Form)
        self.label_8.setMaximumSize(QtCore.QSize(54, 16777215))
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.gridLayout_6.addWidget(self.label_8, 0, 3, 1, 1)
        self.label_7 = QtWidgets.QLabel(Form)
        self.label_7.setMaximumSize(QtCore.QSize(54, 16777215))
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.gridLayout_6.addWidget(self.label_7, 0, 2, 1, 1)
        self.label_9 = QtWidgets.QLabel(Form)
        self.label_9.setMaximumSize(QtCore.QSize(41, 16777215))
        self.label_9.setObjectName("label_9")
        self.gridLayout_6.addWidget(self.label_9, 2, 0, 1, 1)
        self.avgDepthXValue = QtWidgets.QLineEdit(Form)
        self.avgDepthXValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.avgDepthXValue.setObjectName("avgDepthXValue")
        self.gridLayout_6.addWidget(self.avgDepthXValue, 1, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_6)
        self.solvePnpEnableBtn = QtWidgets.QCheckBox(Form)
        self.solvePnpEnableBtn.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.solvePnpEnableBtn.sizePolicy().hasHeightForWidth())
        self.solvePnpEnableBtn.setSizePolicy(sizePolicy)
        self.solvePnpEnableBtn.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.solvePnpEnableBtn.setChecked(True)
        self.solvePnpEnableBtn.setObjectName("solvePnpEnableBtn")
        self.verticalLayout.addWidget(self.solvePnpEnableBtn, 0, QtCore.Qt.AlignHCenter)
        self.gridLayout_8 = QtWidgets.QGridLayout()
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.label_14 = QtWidgets.QLabel(Form)
        self.label_14.setMaximumSize(QtCore.QSize(54, 16777215))
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.gridLayout_8.addWidget(self.label_14, 1, 2, 1, 1)
        self.label_15 = QtWidgets.QLabel(Form)
        self.label_15.setMaximumSize(QtCore.QSize(54, 16777215))
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.gridLayout_8.addWidget(self.label_15, 1, 3, 1, 1)
        self.label_11 = QtWidgets.QLabel(Form)
        self.label_11.setMaximumSize(QtCore.QSize(41, 16777215))
        self.label_11.setObjectName("label_11")
        self.gridLayout_8.addWidget(self.label_11, 2, 0, 1, 1)
        self.stdPnpZValue = QtWidgets.QLineEdit(Form)
        self.stdPnpZValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.stdPnpZValue.setObjectName("stdPnpZValue")
        self.gridLayout_8.addWidget(self.stdPnpZValue, 3, 3, 1, 1)
        self.avgPnpXValue = QtWidgets.QLineEdit(Form)
        self.avgPnpXValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.avgPnpXValue.setObjectName("avgPnpXValue")
        self.gridLayout_8.addWidget(self.avgPnpXValue, 2, 1, 1, 1)
        self.stdPnpYValue = QtWidgets.QLineEdit(Form)
        self.stdPnpYValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.stdPnpYValue.setObjectName("stdPnpYValue")
        self.gridLayout_8.addWidget(self.stdPnpYValue, 3, 2, 1, 1)
        self.label_13 = QtWidgets.QLabel(Form)
        self.label_13.setMaximumSize(QtCore.QSize(54, 16777215))
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.gridLayout_8.addWidget(self.label_13, 1, 1, 1, 1)
        self.avgPnpZValue = QtWidgets.QLineEdit(Form)
        self.avgPnpZValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.avgPnpZValue.setObjectName("avgPnpZValue")
        self.gridLayout_8.addWidget(self.avgPnpZValue, 2, 3, 1, 1)
        self.avgPnpYValue = QtWidgets.QLineEdit(Form)
        self.avgPnpYValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.avgPnpYValue.setObjectName("avgPnpYValue")
        self.gridLayout_8.addWidget(self.avgPnpYValue, 2, 2, 1, 1)
        self.label_12 = QtWidgets.QLabel(Form)
        self.label_12.setMaximumSize(QtCore.QSize(41, 16777215))
        self.label_12.setObjectName("label_12")
        self.gridLayout_8.addWidget(self.label_12, 3, 0, 1, 1)
        self.stdPnpXValue = QtWidgets.QLineEdit(Form)
        self.stdPnpXValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.stdPnpXValue.setObjectName("stdPnpXValue")
        self.gridLayout_8.addWidget(self.stdPnpXValue, 3, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_8)
        self.label_16 = QtWidgets.QLabel(Form)
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.verticalLayout.addWidget(self.label_16)
        self.gridLayout_10 = QtWidgets.QGridLayout()
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.metersSelectBtn = QtWidgets.QRadioButton(Form)
        self.metersSelectBtn.setChecked(True)
        self.metersSelectBtn.setObjectName("metersSelectBtn")
        self.unitsButtonGroup = QtWidgets.QButtonGroup(Form)
        self.unitsButtonGroup.setObjectName("unitsButtonGroup")
        self.unitsButtonGroup.addButton(self.metersSelectBtn)
        self.gridLayout_10.addWidget(self.metersSelectBtn, 0, 0, 1, 1)
        self.feetSelectBtn = QtWidgets.QRadioButton(Form)
        self.feetSelectBtn.setObjectName("feetSelectBtn")
        self.unitsButtonGroup.addButton(self.feetSelectBtn)
        self.gridLayout_10.addWidget(self.feetSelectBtn, 0, 1, 1, 1)
        self.inchesSelectBtn = QtWidgets.QRadioButton(Form)
        self.inchesSelectBtn.setObjectName("inchesSelectBtn")
        self.unitsButtonGroup.addButton(self.inchesSelectBtn)
        self.gridLayout_10.addWidget(self.inchesSelectBtn, 0, 2, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_10)
        self.line_3 = QtWidgets.QFrame(Form)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout.addWidget(self.line_3)
        self.label_10 = QtWidgets.QLabel(Form)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.verticalLayout.addWidget(self.label_10)
        self.pauseResumeBtn = QtWidgets.QPushButton(Form)
        self.pauseResumeBtn.setMaximumSize(QtCore.QSize(223, 16777215))
        self.pauseResumeBtn.setObjectName("pauseResumeBtn")
        self.verticalLayout.addWidget(self.pauseResumeBtn, 0, QtCore.Qt.AlignHCenter)
        self.gridLayout_11 = QtWidgets.QGridLayout()
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.exposureTimeLabel = QtWidgets.QLabel(Form)
        self.exposureTimeLabel.setObjectName("exposureTimeLabel")
        self.gridLayout_11.addWidget(self.exposureTimeLabel, 0, 0, 1, 1)
        self.exposureIsoLabel = QtWidgets.QLabel(Form)
        self.exposureIsoLabel.setObjectName("exposureIsoLabel")
        self.gridLayout_11.addWidget(self.exposureIsoLabel, 1, 0, 1, 1)
        self.exposureIsoSlider = QtWidgets.QSlider(Form)
        self.exposureIsoSlider.setOrientation(QtCore.Qt.Horizontal)
        self.exposureIsoSlider.setObjectName("exposureIsoSlider")
        self.gridLayout_11.addWidget(self.exposureIsoSlider, 1, 1, 1, 1)
        self.exposureIsoValue = QtWidgets.QLineEdit(Form)
        self.exposureIsoValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.exposureIsoValue.setObjectName("exposureIsoValue")
        self.gridLayout_11.addWidget(self.exposureIsoValue, 1, 2, 1, 1)
        self.brightnessLabel = QtWidgets.QLabel(Form)
        self.brightnessLabel.setObjectName("brightnessLabel")
        self.gridLayout_11.addWidget(self.brightnessLabel, 3, 0, 1, 1)
        self.exposureTimeSlider = QtWidgets.QSlider(Form)
        self.exposureTimeSlider.setEnabled(True)
        self.exposureTimeSlider.setAutoFillBackground(False)
        self.exposureTimeSlider.setOrientation(QtCore.Qt.Horizontal)
        self.exposureTimeSlider.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.exposureTimeSlider.setObjectName("exposureTimeSlider")
        self.gridLayout_11.addWidget(self.exposureTimeSlider, 0, 1, 1, 1)
        self.exposureTimeValue = QtWidgets.QLineEdit(Form)
        self.exposureTimeValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.exposureTimeValue.setObjectName("exposureTimeValue")
        self.gridLayout_11.addWidget(self.exposureTimeValue, 0, 2, 1, 1)
        self.whiteBalanceLabel = QtWidgets.QLabel(Form)
        self.whiteBalanceLabel.setObjectName("whiteBalanceLabel")
        self.gridLayout_11.addWidget(self.whiteBalanceLabel, 2, 0, 1, 1)
        self.brightnessSlider = QtWidgets.QSlider(Form)
        self.brightnessSlider.setOrientation(QtCore.Qt.Horizontal)
        self.brightnessSlider.setObjectName("brightnessSlider")
        self.gridLayout_11.addWidget(self.brightnessSlider, 3, 1, 1, 1)
        self.whiteBalanceSlider = QtWidgets.QSlider(Form)
        self.whiteBalanceSlider.setOrientation(QtCore.Qt.Horizontal)
        self.whiteBalanceSlider.setObjectName("whiteBalanceSlider")
        self.gridLayout_11.addWidget(self.whiteBalanceSlider, 2, 1, 1, 1)
        self.whiteBalanceValue = QtWidgets.QLineEdit(Form)
        self.whiteBalanceValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.whiteBalanceValue.setObjectName("whiteBalanceValue")
        self.gridLayout_11.addWidget(self.whiteBalanceValue, 2, 2, 1, 1)
        self.brightnessValue = QtWidgets.QLineEdit(Form)
        self.brightnessValue.setMaximumSize(QtCore.QSize(54, 16777215))
        self.brightnessValue.setObjectName("brightnessValue")
        self.gridLayout_11.addWidget(self.brightnessValue, 3, 2, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_11)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.gridLayout_12.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.frameWidget = QtWidgets.QTabWidget(Form)
        self.frameWidget.setObjectName("frameWidget")
        self.monoRightView = QtWidgets.QWidget()
        self.monoRightView.setObjectName("monoRightView")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.monoRightView)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.monoFrame = QtWidgets.QLabel(self.monoRightView)
        self.monoFrame.setMinimumSize(QtCore.QSize(640, 360))
        self.monoFrame.setText("")
        self.monoFrame.setObjectName("monoFrame")
        self.gridLayout_9.addWidget(self.monoFrame, 0, 0, 1, 1)
        self.frameWidget.addTab(self.monoRightView, "")
        self.depthView = QtWidgets.QWidget()
        self.depthView.setObjectName("depthView")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.depthView)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.depthFrame = QtWidgets.QLabel(self.depthView)
        self.depthFrame.setMinimumSize(QtCore.QSize(640, 360))
        self.depthFrame.setText("")
        self.depthFrame.setAlignment(QtCore.Qt.AlignCenter)
        self.depthFrame.setObjectName("depthFrame")
        self.gridLayout_2.addWidget(self.depthFrame, 1, 0, 1, 1)
        self.frameWidget.addTab(self.depthView, "")
        self.stackedView = QtWidgets.QWidget()
        self.stackedView.setObjectName("stackedView")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.stackedView)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.monoFrame2 = QtWidgets.QLabel(self.stackedView)
        self.monoFrame2.setText("")
        self.monoFrame2.setAlignment(QtCore.Qt.AlignCenter)
        self.monoFrame2.setObjectName("monoFrame2")
        self.verticalLayout_2.addWidget(self.monoFrame2)
        self.depthFrame2 = QtWidgets.QLabel(self.stackedView)
        self.depthFrame2.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.depthFrame2.setText("")
        self.depthFrame2.setAlignment(QtCore.Qt.AlignCenter)
        self.depthFrame2.setObjectName("depthFrame2")
        self.verticalLayout_2.addWidget(self.depthFrame2)
        self.verticalLayout_3.addLayout(self.verticalLayout_2)
        self.frameWidget.addTab(self.stackedView, "")
        self.gridLayout_12.addWidget(self.frameWidget, 0, 1, 1, 1)

        self.retranslateUi(Form)
        self.frameWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Valid Tags"))
        self.tagCheckbox3.setText(_translate("Form", "Tag 3"))
        self.tagCheckbox5.setText(_translate("Form", "Tag 5"))
        self.tagCheckbox1.setText(_translate("Form", "Tag 1"))
        self.tagCheckbox2.setText(_translate("Form", "Tag 2"))
        self.tagCheckbox4.setText(_translate("Form", "Tag 4"))
        self.tagCheckbox6.setText(_translate("Form", "Tag 6"))
        self.tagCheckbox8.setText(_translate("Form", "Tag 8"))
        self.tagCheckbox7.setText(_translate("Form", "Tag 7"))
        self.pitchLabel.setText(_translate("Form", "Pitch:"))
        self.yawLabel.setText(_translate("Form", "Yaw:"))
        self.resetGyroBtn.setText(_translate("Form", "Reset Gyro"))
        self.label_2.setText(_translate("Form", "Stats"))
        self.label_3.setText(_translate("Form", "Detected Tags"))
        self.label_4.setText(_translate("Form", "DepthAI"))
        self.label_6.setText(_translate("Form", "X"))
        self.label_5.setText(_translate("Form", "Average"))
        self.label_8.setText(_translate("Form", "Z"))
        self.label_7.setText(_translate("Form", "Y"))
        self.label_9.setText(_translate("Form", "Std."))
        self.solvePnpEnableBtn.setText(_translate("Form", "AprilTags Pose"))
        self.label_14.setText(_translate("Form", "Y"))
        self.label_15.setText(_translate("Form", "Z"))
        self.label_11.setText(_translate("Form", "Average"))
        self.label_13.setText(_translate("Form", "X"))
        self.label_12.setText(_translate("Form", "Std."))
        self.label_16.setText(_translate("Form", "Units"))
        self.metersSelectBtn.setText(_translate("Form", "Meters"))
        self.feetSelectBtn.setText(_translate("Form", "Feet"))
        self.inchesSelectBtn.setText(_translate("Form", "Inches"))
        self.label_10.setText(_translate("Form", "Camera Controls"))
        self.pauseResumeBtn.setText(_translate("Form", "Pause"))
        self.exposureTimeLabel.setText(_translate("Form", "Exposure (μs)"))
        self.exposureIsoLabel.setText(_translate("Form", "Exposure (iso)"))
        self.brightnessLabel.setText(_translate("Form", "Brightness"))
        self.whiteBalanceLabel.setText(_translate("Form", "White Balance"))
        self.frameWidget.setTabText(self.frameWidget.indexOf(self.monoRightView), _translate("Form", "Mono"))
        self.frameWidget.setTabText(self.frameWidget.indexOf(self.depthView), _translate("Form", "Depth"))
        self.frameWidget.setTabText(self.frameWidget.indexOf(self.stackedView), _translate("Form", "Dual"))

