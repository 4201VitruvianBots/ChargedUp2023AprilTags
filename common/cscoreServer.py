import cscore as cs
import logging
import threading

import cv2
import numpy as np

log = logging.getLogger(__name__)


class CSCoreServer:

    def __init__(self, name, port=8081, width=1280, height=720, fps=60):
        self.frame = np.zeros(shape=(height, width), dtype=np.uint8)
        self.csCamera = cs.CvSource(name, cs.VideoMode.PixelFormat.kGray, width, height, fps)
        self.csCamera.setConnectionStrategy(cs.VideoSource.ConnectionStrategy.kConnectionKeepOpen)

        th = threading.Thread(target=self.run, daemon=True)
        th.start()

        mjpegServer = cs.MjpegServer("localhost", port)
        mjpegServer.setSource(self.csCamera)

    def setFrame(self, frame):
        self.frame = frame

    def run(self):
        while True:
            try:
                self.csCamera.putFrame(self.frame)
            except Exception as e:
                log.error("Could not send frame")
