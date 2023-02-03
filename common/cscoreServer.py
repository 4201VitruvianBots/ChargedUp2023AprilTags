import socket

import cscore as cs
import logging
import numpy as np
import threading


log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.DEBUG)


class CSCoreServer:

    def __init__(self, name,  ip=None, port=5800, width=640, height=400, fps=30):
        self.frame = np.zeros(shape=(height, width), dtype=np.uint8)
        self.csCamera = cs.CvSource(name, cs.VideoMode.PixelFormat.kGray, width, height, fps)
        self.csCamera.setConnectionStrategy(cs.VideoSource.ConnectionStrategy.kConnectionKeepOpen)
        self.port = port

        try:
            if ip is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))

                self.ip_address = s.getsockname()[0]
            else:
                self.ip_address = ip

            self.mjpegServer = cs.MjpegServer(self.ip_address, self.port)
            self.mjpegServer.setSource(self.csCamera)
            log.info("MJPEG Server started at {}:{}".format(self.mjpegServer.getListenAddress(), self.mjpegServer.getPort()))
        except Exception as e:
            log.error("Error Creating MJPEG Server: {}".format(e))

        th = threading.Thread(target=self.run, daemon=True)
        th.start()

    def setFrame(self, frame):
        self.frame = frame

    def getServerLocation(self):
        return "{}:{}".format(self.ip_address, self.port)

    def run(self):
        while True:
            # log.debug("MJPEG Server Status: {}".format(self.mjpegServer.getLastStatus()))
            try:
                self.csCamera.putFrame(self.frame)
            except Exception as e:
                log.error("Could not send frame")
