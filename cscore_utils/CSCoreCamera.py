import json
import logging
import os
import platform
from threading import Thread

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

        self.camera.setVideoMode(cscore.VideoMode.PixelFormat.kMJPEG, camera_params["width"], camera_params["height"],
                                 camera_params["fps"])
        self.camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen)
        if platform.system() == 'Windows':
            settings = open("utils/{}_config.json".format(self.name))
            self.jsonConfig = json.load(settings)
            control = open("utils/{}_control.json".format(self.name))
            self.jsonControl = json.load(control)

            test = self.camera.setConfigJson(self.jsonConfig)
            if not test:
                log.warning("Camera {} config not applied".format(self.name))
            # self.camera.setExposureAuto(0.25)
            self.camera.setExposureManual(-9)
            self.camera.setExposureHoldCurrent()
        elif platform.system() == 'Linux':
            settings = open("utils/{}_config.json".format(self.name))
            self.jsonConfig = json.load(settings)
            control = open("utils/{}_control.json".format(self.name))
            self.jsonControl = json.load(control)

            test = self.camera.setConfigJson(self.jsonConfig)
            if not test:
                log.warning("Camera {} config not applied".format(self.name))
            self.camera.setExposureManual(-9)
            self.camera.setExposureHoldCurrent()

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

