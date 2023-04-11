import json
import logging
import os
import platform
from threading import Thread
import time

import cscore
import cv2
import numpy as np

from cscore_utils.usbCameraUtils import generateCameraParameters

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.DEBUG)

enable_threading = True


class CSCoreCamera:
    def __init__(self, camera_params, grayscale=False):
        self.name = camera_params["device_id"]
        deviceId = 0
        for caminfo in cscore.UsbCamera.enumerateUsbCameras():
            print("%s: %s (%s)" % (caminfo.dev, caminfo.path, caminfo.name))
            if caminfo.name == self.name:
                deviceId = caminfo.dev
        self.camera = cscore.UsbCamera(self.name, deviceId)

        log.info("Initializing Camera Settings")
        self.camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen)
        videoMode = cscore.VideoMode(cscore.VideoMode.PixelFormat.kMJPEG, 1600, 1200, 50)
        self.camera.setVideoMode(videoMode)
        # if platform.system() == 'Windows':
        #     settings = open("utils/{}_config.json".format(self.name))
        #     self.jsonConfig = json.load(settings)
        #     control = open("utils/{}_control.json".format(self.name))
        #     self.jsonControl = json.load(control)
        #
        #     test = self.camera.setConfigJson(self.jsonConfig)
        #     if not test:
        #         log.warning("Camera {} config not applied".format(self.name))
        #     # self.camera.setExposureAuto(0.25)
        #     self.camera.setExposureManual(-9)
        #     self.camera.setExposureHoldCurrent()
        # elif platform.system() == 'Linux':
        self.camera.getProperty('brightness').set(0)
        self.camera.getProperty('contrast').set(32)
        self.camera.getProperty('saturation').set(64)
        self.camera.getProperty('hue').set(0)
        self.camera.getProperty('white_balance_temperature_auto').set(1)
        self.camera.getProperty('gamma').set(100)
        self.camera.getProperty('gain').set(0)
        self.camera.getProperty('power_line_frequency').set(2)
        self.camera.getProperty('sharpness').set(3)
        self.camera.getProperty('backlight_compensation').set(0)
        self.camera.getProperty('exposure_auto').set(1)
        self.camera.getProperty('exposure_absolute').set(157)
        self.camera.getProperty('exposure_auto_priority').set(1)

        log.info("Initializing CV Sink")
        self.cvSink = cscore.CvSink("{}_cvsink".format(self.name))
        self.cvSink.setSource(self.camera)
        if grayscale:
            self.frame = np.zeros([camera_params['height'], camera_params['width']], dtype=np.uint8)
        else:
            self.frame = np.zeros([camera_params['height'], camera_params['width'], 3], dtype=np.uint8)

        self.timestamp = 0
        if enable_threading:
            thread = Thread(target=self._run, daemon=True)
            thread.start()
        else:
            self._run()

        log.info("Done Setting Up CSCoreCamera")

    def _run(self):
        while True:
            timestamp, frame = self.cvSink.grabFrame(self.frame)

            if timestamp == 0:
                log.error(self.cvSink.getError())
                self.timestamp = 0
                continue
            if len(frame.shape) > 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            self.frame = frame
            self.timestamp = timestamp

    def getFrame(self):
        return self.timestamp, self.frame

    def getCvsink(self):
        return self.cvSink

    def getCamera(self):
        return self.camera

    def getName(self):
        return self.name

    def getConfig(self):
        return self.jsonConfig

    def getControl(self):
        return self.jsonControl


if __name__ == '__main__':
    # Test camera init
    enable_threading = False
    camera_params = generateCameraParameters("OV2311_1")
    CSCoreCamera(camera_params, True)

