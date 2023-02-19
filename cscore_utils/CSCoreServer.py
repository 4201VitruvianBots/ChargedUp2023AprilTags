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

    def __init__(self, camera, ip=None, ports=[5800, 5801], width=640, height=400, fps=30):
        self.frame = np.zeros(shape=(height, width), dtype=np.uint8)

        try:
            if ip is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))

                self.ip_address = s.getsockname()[0]
            else:
                self.ip_address = ip
            self.input_source = ports[0]
            self.output_source = ports[1]

            # clear ports
            # try:
            #     s1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #     s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #     s1.bind((self.ip_address, self.input_source))
            #     s1.close()
            #
            #     s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #     s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #     s2.bind((self.ip_address, self.input_source))
            #     s2.close()
            #
            # except Exception as e:
            #     pass

            self.mjpegServer = cs.MjpegServer(self.ip_address, self.input_source)
            self.mjpegServer.setSource(camera.getCamera())
            # test = self.mjpegServer.setConfigJson(camera.getControl())
            # if not test:
            #     log.warning("Camera {} stream config not applied".format(self.name))
            # log.info("MJPEG Server started at {}:{}".format(self.mjpegServer.getListenAddress(), self.mjpegServer.getPort()))

            self.cvSource = cscore.CvSource("{}_cvsource".format(camera.getName()), cs.VideoMode.PixelFormat.kMJPEG, width, height, fps)
            self.cvMjpegServer = cs.MjpegServer(self.ip_address, self.output_source)
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
        return "{}:{}".format(self.ip_address, self.input_source)

    def run(self):
        while True:
            # log.debug("MJPEG Server Status: {}".format(self.mjpegServer.getLastStatus()))
            try:
                self.cvSource.putFrame(self.frame)
            except Exception as e:
                log.error("Could not send frame")
