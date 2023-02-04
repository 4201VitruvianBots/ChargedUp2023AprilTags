import cv2

camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
camera.set(cv2.CAP_PROP_FPS, 50)
camera.set(cv2.CAP_PROP_GAIN, 100)
camera.set(cv2.CAP_PROP_EXPOSURE, -11)
camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
camera.set(cv2.CAP_PROP_SHARPNESS, 0)

