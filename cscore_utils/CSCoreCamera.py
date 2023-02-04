import json
import logging
from threading import Thread

import cscore
import cv2
import numpy as np

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.DEBUG)

enable_threading = True


class CSCoreCamera:
    def __init__(self, name, grayscale=False):
        self.name = name
        deviceId = 0
        for caminfo in cscore.UsbCamera.enumerateUsbCameras():
            print("%s: %s (%s)" % (caminfo.dev, caminfo.path, caminfo.name))
            if caminfo.name == self.name:
                deviceId = caminfo.dev
        self.camera = cscore.UsbCamera(self.name, deviceId)
        settings = open("utils/{}.json".format(self.name))
        jsonData = json.load(settings)
        # test = self.camera.setConfigJson(jsonData)
        # if not test:
        #     log.warning("Camera {} settings not applied".format(name))
        self.camera.setVideoMode(cscore.VideoMode.PixelFormat.kMJPEG, 1600, 1200, 50)
        self.camera.setBrightness(0)
        self.camera.setWhiteBalanceManual(0)
        self.camera.setExposureManual(10)
        self.camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen)

        self.cvSink = cscore.CvSink("{}_cvsink".format(self.name))
        self.cvSink.setSource(self.camera)
        if grayscale:
            self.frame = np.zeros([jsonData['height'], jsonData['width']], dtype=np.uint8)
        else:
            self.frame = np.zeros([jsonData['height'], jsonData['width'], 3], dtype=np.uint8)

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

    def getName(self):
        return self.name


if __name__ == '__main__':
    # Test camera init
    enable_threading = False
    CSCoreCamera("usbcam", True)

