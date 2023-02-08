import json
import socket

import cscore
import cscore as cs
import logging
import numpy as np
import threading


log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.DEBUG)


class CSCoreServer:

    def __init__(self, camera, ip=None, port=5800, width=640, height=400, fps=30):
        self.frame = np.zeros(shape=(height, width), dtype=np.uint8)

        try:
            if ip is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))

                self.ip_address = s.getsockname()[0]
            else:
                self.ip_address = ip
            self.port = port

            self.mjpegServer = cs.MjpegServer(self.ip_address, self.port)
            self.mjpegServer.setSource(camera.getCamera())
            test = self.mjpegServer.setConfigJson(camera.getConfig())
            if not test:
                log.warning("Camera {} stream config not applied".format(self.name))
            log.info("MJPEG Server started at {}:{}".format(self.mjpegServer.getListenAddress(), self.mjpegServer.getPort()))

            self.cvSource = cscore.CvSource("{}_cvsource".format(camera.getName()), cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)
            self.cvMjpegServer = cs.MjpegServer(self.ip_address, self.port + 1)
            self.cvMjpegServer.setSource(self.cvSource)
            cs.CameraServer.addServer(self.mjpegServer)
            cs.CameraServer.addServer(self.cvMjpegServer)
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
                self.cvSource.putFrame(self.frame)
            except Exception as e:
                log.error("Could not send frame")
