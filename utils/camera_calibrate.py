import argparse
import copy
import datetime
import glob
import os
import platform

from typing import List

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import numpy

parser = argparse.ArgumentParser()
parser.add_argument('-d', dest='image_directory', action="store", type=str, help='Use images in given directory')

args = parser.parse_args()

cap_props = [x for x in dir(cv2) if x.startswith('CAP_PROP_')]


class CameraCalibrate:
    _all_charuco_corners: List[numpy.ndarray] = []
    _all_charuco_ids: List[numpy.ndarray] = []
    _imsize = None

    def __init__(self, device_name) -> None:
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        # Squares X, Squares Y, Squares L, Markers  L
        # self._charuco_board = cv2.aruco.CharucoBoard((12, 9), 0.030, 0.023, self._aruco_dict)
        self._charuco_board = cv2.aruco.CharucoBoard((12, 9), 0.021, 0.016, self._aruco_dict)
        self._device_name = device_name
        self._filename = "{}.csv".format(device_name)
        self._detector = cv2.aruco.CharucoDetector(self._charuco_board)

    def process_frame(self, image: cv2.Mat, save: bool) -> None:
        # Get image size
        if self._imsize == None:
            self._imsize = (image.shape[0], image.shape[1])

        # Detect tags
        (charucoCorners, charucoIds, makerCorners, markerIds) = self._detector.detectBoard(image)
        if len(makerCorners) > 0:
            cv2.aruco.drawDetectedMarkers(image, makerCorners)

            cv2.aruco.drawDetectedCornersCharuco(image, charucoCorners, charucoIds)

            cv2.imshow("Detection Results", image)
            # Save corners
            if save:
                self._all_charuco_corners.append(charucoCorners)
                self._all_charuco_ids.append(charucoIds)
                print("Saved calibration frame")

    def finish(self) -> None:
        if len(self._all_charuco_corners) == 0:
            print("ERROR: No calibration data")
            return

        if os.path.exists(self._filename):
            os.remove(self._filename)

        (retval, camera_matrix, distortion_coefficients, rvecs, tvecs) = cv2.aruco.calibrateCameraCharuco(
            self._all_charuco_corners, self._all_charuco_ids, self._charuco_board, self._imsize, None, None)

        if retval:
            calibration_store = cv2.FileStorage(self._filename, cv2.FILE_STORAGE_WRITE)
            calibration_store.write("calibration_date", str(datetime.datetime.now()))
            calibration_store.write("camera_resolution", self._imsize)
            calibration_store.write("camera_matrix", camera_matrix)
            calibration_store.write("distortion_coefficients", distortion_coefficients)
            calibration_store.release()
            print("Calibration finished")
        else:
            print("ERROR: Calibration failed")


def main():
    camera_calibration = CameraCalibrate("OV2311_1")

    if args.image_directory:
        print("Calibrating from exisiting images")
        searchRegex = r"{}/*.jpg".format(args.image_directory)
        images = glob.glob(searchRegex)

        for image in images:
            camera_calibration.process_frame(image, save=True)
    else:
        camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
        camera.set(cv2.CAP_PROP_FORMAT, -1)
        # camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        camera.set(cv2.CAP_PROP_FPS, 50)
        camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        camera.set(cv2.CAP_PROP_AUTO_WB, 0)
        
        camera.set(cv2.CAP_PROP_WB_TEMPERATURE, 0)
        camera.set(cv2.CAP_PROP_ISO_SPEED, 30)
        camera.set(cv2.CAP_PROP_CONTRAST, 0)
        camera.set(cv2.CAP_PROP_SATURATION, 0)
        camera.set(cv2.CAP_PROP_GAMMA, 0)
        camera.set(cv2.CAP_PROP_GAIN, -1)
        camera.set(cv2.CAP_PROP_EXPOSURE, -15)
        camera.set(cv2.CAP_PROP_FOCUS, 0)
        camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
        camera.set(cv2.CAP_PROP_SHARPNESS, 0)

        current_prop_value = {}
        for prop in cap_props:
            current_prop_value[prop] = camera.get(getattr(cv2, prop))

        maxSamples = 20
        sampleCount = 0
        waitForInput = False

        while sampleCount < maxSamples:
            retVal, frame = camera.read(0)

            for prop in cap_props:
                if current_prop_value[prop] != camera.get(getattr(cv2, prop)):
                    print("WARNING: {} has changed".format(prop))

            if retVal:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                cv2.imshow("Frame", frame)
                # print("Exposure: {}\tWhite Balance: {}".format(camera.get(cv2.CAP_PROP_EXPOSURE), camera.get(cv2.CAP_PROP_WB_TEMPERATURE)))
                key = cv2.waitKey(1)
                if key == ord(' '):
                    print("Image Captured. Do you want to use it (S: Save)?")
                    waitForInput = True
                    rawFrame = copy.copy(frame)
                    camera_calibration.process_frame(frame, save=False)

                while waitForInput:
                    key = cv2.waitKey(1)
                    if key == ord('s'):
                        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                        if platform.system() == 'Windows':
                            dir = os.path.join(os.getcwd(), "resources\\calibration_images")
                        else:
                            dir = os.path.join(os.getcwd(), "resources/calibration_images")
                        if not os.path.isdir(dir):                            os.mkdir(dir)

                        if platform.system() == 'Windows':
                            filename = "{}\\{}{}.jpg".format(dir, timestamp, "OV2311_1")
                            cv2.imwrite(filename, rawFrame)
                        else:
                            filename = "{}/{}-{}.jpg".format(dir, timestamp, "OV2311_1")
                            cv2.imwrite(filename, rawFrame)
                        print(filename)
                        camera_calibration.process_frame(frame, save=True)
                        sampleCount += 1
                        waitForInput = False
                        try:
                            cv2.destroyWindow("Detection Results")
                        except Exception as e:
                            pass
                    elif key == ord('r'):
                        waitForInput = False
                        try:
                            cv2.destroyWindow("Detection Results")
                        except Exception as e:
                            pass
                    else:
                        pass

    camera_calibration.finish()


if __name__ == '__main__':
    main()
