import cscore as cs

camera = cs.UsbCamera("usbcam", 0)
camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 1600, 1200, 50)

mjpegServer = cs.MjpegServer("localhost", 8081)
mjpegServer.setSource(camera)

print("mjpg server listening at http://0.0.0.0:8081")
input("Press enter to exit...")