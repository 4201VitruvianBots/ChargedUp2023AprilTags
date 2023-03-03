import os
import socket
from os.path import basename

import cscore
import ntcore


def main():
    ip = None
    try:
        if ip is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))

            ip_address = s.getsockname()[0]
        else:
            ip_address = ip
    except Exception as e:
        print("Error Creating MJPEG Server: {}".format(e))

    cameraValues = [
        {
            'name': 'OV2311_1',
            'deviceID': 0,
            'height': 320,  # 1600, 1280, 1280, 800, 640, 320, 160
            'width': 240,   # 1200,  960,  720, 600, 320, 240, 120
            'fps': 50
        },
        {
            'name': 'OV2311_2',
            'deviceID': 1,
            'height': 320,  # 1600, 1280, 1280, 800, 640, 320, 160
            'width': 240,   # 1200,  960,  720, 600, 320, 240, 120
            'fps': 50
        },
    ]
    cameraList = []
    cameraServerList = []

    basePortNum = 5800
    for i in range(len(cameraValues)):
        camera = cscore.UsbCamera(cameraValues[i]['name'], cameraValues[i]['deviceID'])

        camera.setVideoMode(cscore.VideoMode.PixelFormat.kMJPEG,
                            cameraValues[i]["width"],
                            cameraValues[i]["height"],
                            cameraValues[i]["fps"])

        camera.setConnectionStrategy(cscore.VideoCamera.ConnectionStrategy.kConnectionKeepOpen)

        cameraList.append(camera)
        cameraServer = cscore.MjpegServer(ip_address, basePortNum + i)
        cameraServer.setSource(camera)
        cameraServerList.append(cameraServer)

        cscore.CameraServer.addServer(cameraServer)

    nt_instance = ntcore.NetworkTableInstance.getDefault()
    nt_instance.setServerTeam(4201)
    identity = f"{basename(__file__)}-{os.getpid()}"
    nt_instance.startClient4(identity)
    cscore.CameraServer.waitForever()


if __name__ == '__main__':
    main()
