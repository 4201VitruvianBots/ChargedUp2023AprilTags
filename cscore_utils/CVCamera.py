import json
import logging
import os
import platform
import queue
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


class CVCamera:
    def __init__(self, camera_params, enable_threading=True):
        self.name = camera_params["device_id"]
        deviceId = 0

        self.camera = cscore.CvSource("cvsource", cscore.VideoMode.PixelFormat.kMJPEG, camera_params['width'], camera_params['height'], camera_params['fps'])

        log.info("Setting up camera settings...")
        self.FPS_MS = int(1/camera_params['fps'] * 1000)
        if platform.system() == 'Windows':
            self.cap = cv2.VideoCapture(deviceId)

            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_params['height'])
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_params['width'])
            self.cap.set(cv2.CAP_PROP_FPS, camera_params['fps'])
            # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('m', 'j', 'p', 'g'))
            # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            # self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
            # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            # self.cap.set(cv2.CAP_PROP_EXPOSURE, 157)
            # self.cap.set(cv2.CAP_PROP_GAIN, 0)
            # self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
            # self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
            # self.cap.set(cv2.CAP_PROP_CONTRAST, 32)
            # self.cap.set(cv2.CAP_PROP_GAMMA, 100)
            # self.cap.set(cv2.CAP_PROP_HUE, 0)
            # self.cap.set(cv2.CAP_PROP_SATURATION, 64)
        else:
            # self.cap = cv2.VideoCapture("v4l2src device=/dev/video" + str(deviceId) +
            #                             " extra_controls=\"c,exposure_auto=" + str(1) +
            #                             ",exposure_absolute=" + str(157) +
            #                             ",gain=" + str(0) + ",sharpness=0,brightness=0\"" +
            #                             "! image/jpeg,format=MJPG,width=" + str(camera_params['width']) +
            #                             ",height=" + str(camera_params['height']) +
            #                             " ! jpegdec ! video/x-raw ! appsink drop=1", cv2.CAP_GSTREAMER)
            self.cap = cv2.VideoCapture(deviceId, cv2.CAP_V4L)

            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_params['height'])
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_params['width'])
            self.cap.set(cv2.CAP_PROP_FPS, camera_params['fps'])
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('m', 'j', 'p', 'g'))
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, 20)
            self.cap.set(cv2.CAP_PROP_GAIN, 0)
            self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
            self.cap.set(cv2.CAP_PROP_CONTRAST, 32)
            self.cap.set(cv2.CAP_PROP_GAMMA, 0)
            self.cap.set(cv2.CAP_PROP_HUE, 0)
            self.cap.set(cv2.CAP_PROP_SATURATION, 64)

        self.frame = np.zeros([camera_params['height'], camera_params['width'], 3], dtype=np.uint8)

        self.timestamp = 0
        self.frameQueue = queue.Queue(3)
        self.fpsHandler = FPSHandler()
        if enable_threading:
            thread = Thread(target=self._run, daemon=True)
            thread.start()
        else:
            self._run()

        log.info("Done Setting Up CSCoreCamera")

    def _run(self):
        while True:
            if self.cap.isOpened():
                retval, frame = self.cap.read(self.frame)
                timestamp = time.time()

                if retval:
                    self.fpsHandler.nextIter()
                    log.debug("FPS: {}".format(self.fpsHandler.fps()))
                    try:
                        if self.frameQueue.full():
                            if self.frameQueue.full():
                                dropFrame = self.frameQueue.get(block=False, timeout=0.02)
                            else:
                                self.frameQueue.put((timestamp, frame), block=False, timeout=0.02)
                        else:
                            self.frameQueue.put((timestamp, frame), block=False, timeout=0.02)
                    except Exception as e:
                        log.debug("Frame queue error")
                else:
                    log.error("Error capturing frames")
                # time.sleep(self.FPS_MS)

    def getFrame(self):
        return self.frameQueue.get()

    def getCamera(self):
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
    videoMode = cscore.VideoMode(cscore.VideoMode.PixelFormat.kMJPEG, 1600, 1200, 50)
    cvSource = cscore.CvSource("cvsource", videoMode)
    mjpegServer.setSource(cvSource)

    cscore.CameraServer.addServer(mjpegServer)
    print("Start Capture...")
    while True:
        timestamp, frame = camera.getFrame()
        cvSource.putFrame(frame)
