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


class CVCamera:
    def __init__(self, camera_params, enable_threading=True):
        self.name = camera_params["device_id"]
        deviceId = 0

        self.camera = cscore.CvSource("cvsource", cscore.VideoMode.PixelFormat.kMJPEG, camera_params['width'], camera_params['height'], camera_params['fps'])

        log.info("Setting up camera settings...")
        if platform.system() == 'Windows':
            self.cap = cv2.VideoCapture(deviceId)

            self.cap.set(cv2.CAP_PROP_FPS, camera_params['fps'])
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, 157)
            self.cap.set(cv2.CAP_PROP_GAIN, 0)
            self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
            self.cap.set(cv2.CAP_PROP_CONTRAST, 32)
            self.cap.set(cv2.CAP_PROP_GAMMA, 100)
            self.cap.set(cv2.CAP_PROP_HUE, 0)
            self.cap.set(cv2.CAP_PROP_SATURATION, 64)
        else:
            self.cap = cv2.VideoCapture("v4l2src device=/dev/video" + str(deviceId) +
                                        " extra_controls=\"c,exposure_auto=" + str(1) +
                                        ",exposure_absolute=" + str(157) +
                                        ",gain=" + str(0) + ",sharpness=0,brightness=0\"" +
                                        "! image/jpeg,format=MJPG,width=" + str(camera_params['width']) +
                                        ",height=" + str(camera_params['height']) +
                                        " ! jpegdec ! video/x-raw ! appsink drop=1", cv2.CAP_GSTREAMER)

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
            retval, frame = self.cap.read(self.frame)
            timestamp = time.time()

            if retval:
                self.frame = frame
                self.timestamp = timestamp
            else:
                log.error("Error capturing frames")

    def getFrame(self):
        return self.timestamp, self.frame

    def getCvSource(self):
        return self.camera

    def getName(self):
        return self.name


if __name__ == '__main__':
    # Test camera init
    enable_threading = True
    camera_params = generateCameraParameters("OV2311_1")
    camera = CVCamera(camera_params, enable_threading)

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
    print("Start Capture...")
    while True:
        timestamp, frame = camera.getFrame()
        cvSource.putFrame(frame)
