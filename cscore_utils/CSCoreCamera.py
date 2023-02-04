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


class CSCoreCamera:
    def __init__(self, name, grayscale=False):
        deviceId = 0
        for caminfo in cscore.UsbCamera.enumerateUsbCameras():
            print("%s: %s (%s)" % (caminfo.dev, caminfo.path, caminfo.name))
            if caminfo.name == name:
                deviceId = caminfo.dev
        self.camera = cscore.UsbCamera(name, deviceId)
        settings = open("utils/{}.json".format(name))
        jsonData = json.load(settings)
        test = self.camera.setConfigJson(jsonData)
        if not test:
            log.warning("Camera {} settings not applied".format(name))
        self.camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen);
        self.cvsink = cscore.CvSink("{}_cvsink".format(name))
        self.cvsink.setSource(self.camera)

        if grayscale:
            self.frame = np.zeros([jsonData['height'], jsonData['width']], dtype=np.uint8)
        else:
            self.frame = np.zeros([jsonData['height'], jsonData['width'], 3], dtype=np.uint8)

        self.timestamp = 0
        thread = Thread(target=self._run)
        thread.setDaemon(True)
        thread.start()

    def _run(self):
        while True:
            timestamp, frame = self.cvsink.grabFrame(self.frame)

            if timestamp == 0:
                log.error(self.cvsink.getError())
                self.timestamp = 0
                continue
            if len(frame.shape) > 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            self.frame = frame
            self.timestamp = timestamp

    def getFrame(self):
        return self.timestamp, self.frame

    def getCvsink(self):
        return self.cvsink


if __name__ == '__main__':
    CscoreCamera("usbcam", True)
