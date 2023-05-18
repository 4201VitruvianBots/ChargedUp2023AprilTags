import json
import logging
import os
import platform
from threading import Thread
import time

import cscore
import cv2
import numpy as np

from common.utils import FPSHandler
from cscore_utils.usbCameraUtils import generateCameraParameters

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.DEBUG)


class CSCoreCamera:
    def __init__(self, camera_params, enable_threading=True):
        self.name = camera_params["device_id"]
        deviceId = 1
        for caminfo in cscore.UsbCamera.enumerateUsbCameras():
            print("%s: %s (%s)" % (caminfo.dev, caminfo.path, caminfo.name))
            if caminfo.name == self.name:
                deviceId = caminfo.dev
        self.camera = cscore.UsbCamera(self.name, deviceId)

        log.info("Initializing Camera Settings")
        self.camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen)
        self.camera.setVideoMode(
            cscore.VideoMode(cscore.VideoMode.PixelFormat.kMJPEG, camera_params['height'], camera_params['width'],
                             camera_params['fps']))
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
        self.camera.getProperty('brightness').set(0)  # 0
        self.camera.getProperty('contrast').set(32)  # 32
        self.camera.getProperty('saturation').set(64)  # 64
        self.camera.getProperty('hue').set(0)  # 0
        self.camera.getProperty('white_balance_temperature_auto').set(1)  # 1
        self.camera.getProperty('gamma').set(100)  # 100
        self.camera.getProperty('gain').set(0)  # 0
        self.camera.getProperty('power_line_frequency').set(2)  # 2
        self.camera.getProperty('sharpness').set(3)  # 3
        self.camera.getProperty('backlight_compensation').set(0)  # 0
        self.camera.getProperty('exposure_auto').set(1)  # 1
        self.camera.getProperty('exposure_absolute').set(157)  # 157
        self.camera.getProperty('exposure_auto_priority').set(1)  # 1
        # brightness: 84
        # contrast: 29
        # sharpness: 96
        # saturation: 9

        log.info("Initializing CV Sink")
        self.cvSink = cscore.CvSink("{}_cvsink".format(self.name))
        self.cvSink.setSource(self.camera)
         # self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        # self.cap.set(cv2.CAP_PROP_FPS, 50)

        self.frame = np.zeros([camera_params['height'], camera_params['width'], 3], dtype=np.uint8)

        self.timestamp = 0
        self.fpsHandler = FPSHandler()
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

            self.fpsHandler.nextIter()
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

    def getFpsCounter(self):
        return self.fpsHandler


if __name__ == '__main__':
    # Test camera init
    enable_threading = True
    camera_params = generateCameraParameters("OV2311_1")
    camera = CSCoreCamera(camera_params, enable_threading)

    # while True:
    #     timestamp, test = camera.getFrame()
    #     cv2.imshow("Test", test)
    #
    #     key = cv2.waitKey(1)
    #     if key == ord('q'):
    #         break

    mjpegServer = cscore.MjpegServer("test", 5800)
    cvSource = cscore.CvSource("cvsource", cscore.VideoMode.PixelFormat.kMJPEG, 1600, 1200, 50)
    mjpegServer.setSource(cvSource)

    cscore.CameraServer.addServer(mjpegServer)
    test = np.zeros(shape=(1600, 1200, 3), dtype=np.uint8)
    print("Start Capture...")
    while True:
        time, test = camera.getCvsink().grabFrame(test)
        if time == 0:
            print("error:", camera.getCvsink().getError())
            continue

        # print("got frame at time", time, test.shape)
        log.debug("FPS: {:02f}".format(camera.getFpsCounter().fps()))
        cvSource.putFrame(test)
