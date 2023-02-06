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
    def __init__(self, camera_params, grayscale=False):
        self.name = camera_params["device_id"]
        deviceId = 0
        for caminfo in cscore.UsbCamera.enumerateUsbCameras():
            print("%s: %s (%s)" % (caminfo.dev, caminfo.path, caminfo.name))
            if caminfo.name == self.name:
                deviceId = caminfo.dev
        self.camera = cscore.UsbCamera(self.name, deviceId)
        settings = open("utils/csSettings.json")
        self.jsonConfig = json.load(settings)
        self.cameraSettings = self.jsonConfig["cameras"]
        for settings in self.cameraSettings:
            if settings["name"] == self.name:
                self.cameraSettings = settings
                self.streamSettings = settings["stream"]

        test = self.camera.setConfigJson(self.cameraSettings)
        if not test:
            log.warning("Camera {} config not applied".format(self.name))
        self.camera.setVideoMode(cscore.VideoMode.PixelFormat.kMJPEG, camera_params["width"], camera_params["height"], camera_params["fps"])

        self.camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen)

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

    def getName(self):
        return self.name

    def getStreamSettings(self):
        return self.streamSettings


if __name__ == '__main__':
    # Test camera init
    enable_threading = False
    CSCoreCamera("usbcam", True)

