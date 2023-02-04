import sys

import cv2
import numpy as np
from PyQt5 import QtGui, QtWidgets, uic
from PyQt5.QtGui import QIntValidator


class DebugWindow(QtWidgets.QWidget):
    def __init__(self, gyro=None, solvePnp=False):
        super(DebugWindow, self).__init__()
        uic.loadUi('designer/debugWindow.ui', self)
        self.tagFilter = list(range(1, 9))
        self.tagCheckbox1.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox2.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox3.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox4.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox5.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox6.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox7.stateChanged.connect(lambda: self.updateTagFilter())
        self.tagCheckbox8.stateChanged.connect(lambda: self.updateTagFilter())

        self.gyro = gyro
        if self.gyro is not None:
            self.resetGyroBtn.clicked.connect(lambda: self.resetGyroButtonPressed(self.gyro))
        else:
            self.resetGyroBtn.setEnabled(False)
            self.yawValue.setEnabled(False)
            self.pitchValue.setEnabled(False)

        self.values = {
            'numTags': 0,
            'depthAI': {
                'x_pos': [0],
                'y_pos': [0],
                'z_pos': [0],
            },
            'solvePnP': {
                'x_pos': [0],
                'y_pos': [0],
                'z_pos': [0],
            }
        }
        self.unitsButtonGroup.buttonClicked.connect(lambda: self.updateUnits())
        self.unitScale = 1.0

        self.solvePnp = solvePnp
        self.solvePnpEnableBtn.setChecked(self.solvePnp)
        self.solvePnpEnableBtn.clicked.connect(lambda: self.toggleSolvePnp())

        self.camera_settings = {
            'manual_exposure_usec': 2000,
            'manual_exposure_iso': 200,
            'brightness': 5,
            'white_balance': 6000
        }

        self.initializeCameraSettings()
        self.lockCameraUpdates = False

        self.exposureTimeSlider.valueChanged.connect(lambda: self.updateCameraSettings(True))
        # self.exposureTimeValue.textChanged.connect(lambda: self.updateCameraSettings(False))
        self.exposureIsoSlider.valueChanged.connect(lambda: self.updateCameraSettings(True))
        # self.exposureIsoValue.textChanged.connect(lambda: self.updateCameraSettings(False))
        self.brightnessSlider.valueChanged.connect(lambda: self.updateCameraSettings(True))
        # self.brightnessValue.textChanged.connect(lambda: self.updateCameraSettings(False))
        self.whiteBalanceSlider.valueChanged.connect(lambda: self.updateCameraSettings(True))
        # self.whiteBalanceValue.textChanged.connect(lambda: self.updateCameraSettings(False))

        self.pauseResumeBtn.clicked.connect(lambda: self.pauseResumeButtonPressed())
        self.pause = False
        self.show()

    def updateYawValue(self, value):
        self.yawValue.setText("{:.06f}".format(value))

    def updatePitchValue(self, value):
        self.pitchValue.setText("{:.06f}".format(value))

    def updateTagIds(self, tagIds):
        if len(tagIds) > 0:
            self.detectedTagsValue.setText("{}".format(tagIds))
        else:
            self.detectedTagsValue.setText('')

    def updateStatsValue(self, values=None):
        if values is None:
            values = self.values

        self.avgDepthXValue.setText("{:.04f}".format(np.average(values['depthAI']['x_pos']) * self.unitScale))
        self.avgDepthYValue.setText("{:.04f}".format(np.average(values['depthAI']['y_pos']) * self.unitScale))
        self.avgDepthZValue.setText("{:.04f}".format(np.average(values['depthAI']['z_pos']) * self.unitScale))
        self.stdDepthXValue.setText("{:.04f}".format(np.std(values['depthAI']['x_pos']) * self.unitScale))
        self.stdDepthYValue.setText("{:.04f}".format(np.std(values['depthAI']['y_pos']) * self.unitScale))
        self.stdDepthZValue.setText("{:.04f}".format(np.std(values['depthAI']['z_pos']) * self.unitScale))

        self.avgPnpXValue.setText("{:.04f}".format(np.average(values['solvePnP']['x_pos']) * self.unitScale))
        self.avgPnpYValue.setText("{:.04f}".format(np.average(values['solvePnP']['y_pos']) * self.unitScale))
        self.avgPnpZValue.setText("{:.04f}".format(np.average(values['solvePnP']['z_pos']) * self.unitScale))
        self.stdPnpXValue.setText("{:.04f}".format(np.std(values['solvePnP']['x_pos']) * self.unitScale))
        self.stdPnpYValue.setText("{:.04f}".format(np.std(values['solvePnP']['y_pos']) * self.unitScale))
        self.stdPnpZValue.setText("{:.04f}".format(np.std(values['solvePnP']['z_pos']) * self.unitScale))
        self.values = values

    def updateFrames(self, monoFrame, depthFrame):
        activeTab = self.frameWidget.currentIndex()

        if activeTab == 0:
            monoFrame = cv2.cvtColor(monoFrame, cv2.COLOR_GRAY2RGB)
            img = QtGui.QImage(monoFrame, monoFrame.shape[1], monoFrame.shape[0], QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(img)
            self.monoFrame.setMinimumWidth(monoFrame.shape[1])
            self.monoFrame.setMinimumHeight(monoFrame.shape[0])
            self.monoFrame.setMaximumWidth(monoFrame.shape[1])
            self.monoFrame.setMaximumHeight(monoFrame.shape[0])
            self.monoFrame.setPixmap(pix)
        elif activeTab == 1:
            depthFrame = cv2.cvtColor(depthFrame, cv2.COLOR_BGR2RGB)
            img = QtGui.QImage(depthFrame, depthFrame.shape[1], depthFrame.shape[0], QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(img)
            self.depthFrame.setMinimumWidth(depthFrame.shape[1])
            self.depthFrame.setMinimumHeight(depthFrame.shape[0])
            self.depthFrame.setMaximumWidth(depthFrame.shape[1])
            self.depthFrame.setMaximumHeight(depthFrame.shape[0])
            self.depthFrame.setPixmap(pix)
        elif activeTab == 2:
            monoFrame = cv2.cvtColor(monoFrame, cv2.COLOR_GRAY2RGB)
            img = QtGui.QImage(monoFrame, monoFrame.shape[1], monoFrame.shape[0], QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(img)
            self.monoFrame2.setMinimumWidth(monoFrame.shape[1])
            self.monoFrame2.setMinimumHeight(monoFrame.shape[0])
            self.monoFrame2.setMaximumWidth(monoFrame.shape[1])
            self.monoFrame2.setMaximumHeight(monoFrame.shape[0])
            self.monoFrame2.setPixmap(pix)

            depthFrame = cv2.cvtColor(depthFrame, cv2.COLOR_BGR2RGB)
            img = QtGui.QImage(depthFrame, depthFrame.shape[1], depthFrame.shape[0], QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(img)
            self.depthFrame2.setMinimumWidth(depthFrame.shape[1])
            self.depthFrame2.setMinimumHeight(depthFrame.shape[0])
            self.depthFrame2.setMaximumWidth(depthFrame.shape[1])
            self.depthFrame2.setMaximumHeight(depthFrame.shape[0])
            self.depthFrame2.setPixmap(pix)

    def updateUnits(self):
        if self.unitsButtonGroup.checkedId() == -2:
            self.unitScale = 1.0
        elif self.unitsButtonGroup.checkedId() == -3:
            self.unitScale = 3.28084
        elif self.unitsButtonGroup.checkedId() == -4:
            self.unitScale = 39.3701

        self.updateStatsValue()

    def updateTagFilter(self):
        filter = np.array(np.where([self.tagCheckbox1.isChecked(), self.tagCheckbox2.isChecked(),
                                    self.tagCheckbox3.isChecked(), self.tagCheckbox4.isChecked(),
                                    self.tagCheckbox5.isChecked(), self.tagCheckbox6.isChecked(),
                                    self.tagCheckbox7.isChecked(), self.tagCheckbox8.isChecked()])) + 1

        self.tagFilter = filter.tolist()[0]

    def getTagFilter(self):
        return self.tagFilter

    def resetGyroButtonPressed(self, gyro):
        if gyro is not None:
            gyro.resetAll()

    def toggleSolvePnp(self):
        self.solvePnp = self.solvePnpEnableBtn.isChecked()

    def getSolvePnpEnabled(self):
        return self.solvePnp

    def pauseResumeButtonPressed(self):
        if self.pause:
            self.pause = False
            self.pauseResumeBtn.setText("Pause")
        else:
            self.pause = True
            self.pauseResumeBtn.setText("Resume")

    def getPauseResumeState(self):
        return self.pause

    def initializeCameraSettings(self):
        # Exposure time (microseconds)
        self.exposureTimeLabel.setToolTip("Min: 0, Max 10000")
        self.exposureTimeSlider.setMinimum(0)
        self.exposureTimeSlider.setMaximum(10000)
        self.exposureTimeSlider.setValue(self.camera_settings['manual_exposure_usec'])
        self.exposureTimeValue.setText(self.camera_settings['manual_exposure_usec'].__str__())
        exposureTimeRange = QIntValidator()
        exposureTimeRange.setRange(0, 10000)
        self.exposureTimeValue.setValidator(exposureTimeRange)

        # Exposure ISO Sensitivity (100, 1600)
        self.exposureIsoLabel.setToolTip("Min: 100, Max 1600")
        self.exposureIsoSlider.setMinimum(100)
        self.exposureIsoSlider.setMaximum(1600)
        self.exposureIsoSlider.setValue(self.camera_settings['manual_exposure_iso'])
        self.exposureIsoValue.setText(self.camera_settings['manual_exposure_iso'].__str__())
        exposureIsoRange = QIntValidator()
        exposureIsoRange.setRange(100, 1600)
        self.exposureIsoValue.setValidator(exposureIsoRange)

        # Temperature in Kelvins (1000, 12000)
        self.whiteBalanceLabel.setToolTip("Min: 1000, Max 12000")
        self.whiteBalanceSlider.setMinimum(1000)
        self.whiteBalanceSlider.setMaximum(12000)
        self.whiteBalanceSlider.setValue(self.camera_settings['white_balance'])
        self.whiteBalanceValue.setText(self.camera_settings['white_balance'].__str__())
        whiteBalanceRange = QIntValidator()
        whiteBalanceRange.setRange(1000, 12000)
        self.whiteBalanceValue.setValidator(whiteBalanceRange)

        # Image Brightness (-10, 10)
        self.brightnessLabel.setToolTip("Min: -10, Max 10")
        self.brightnessSlider.setMinimum(-10)
        self.brightnessSlider.setMaximum(10)
        self.brightnessSlider.setValue(self.camera_settings['brightness'])
        self.brightnessValue.setText(self.camera_settings['brightness'].__str__())
        brightnessRange = QIntValidator()
        brightnessRange.setRange(-10, 10)
        self.brightnessValue.setValidator(brightnessRange)

    def updateCameraSettings(self, slider):
        # if not self.lockCameraUpdates:
        #     self.lockCameraUpdates = True
        if slider:
            self.exposureTimeValue.setText(self.exposureTimeSlider.value().__str__())
            self.exposureIsoValue.setText(self.exposureIsoSlider.value().__str__())
            self.whiteBalanceValue.setText(self.whiteBalanceSlider.value().__str__())
            self.brightnessValue.setText(self.brightnessSlider.value().__str__())
        else:
            try:
                self.exposureTimeSlider.setValue(int(self.exposureTimeValue.text()))
                self.exposureIsoSlider.setValue(int(self.exposureIsoValue.text()))
                self.whiteBalanceSlider.setValue(int(self.whiteBalanceValue.text()))
                self.brightnessSlider.setValue(int(self.brightnessValue.text()))
            except Exception as e:
                self.lockCameraUpdates = False
                return

        self.camera_settings['manual_exposure_usec'] = int(self.exposureTimeValue.text())
        self.camera_settings['manual_exposure_iso'] = int(self.exposureIsoValue.text())
        self.camera_settings['white_balance'] = int(self.whiteBalanceValue.text())
        self.camera_settings['brightness'] = int(self.brightnessValue.text())
        # else:
        #     self.lockCameraUpdates = False

    def getCameraSettings(self):
        return self.camera_settings