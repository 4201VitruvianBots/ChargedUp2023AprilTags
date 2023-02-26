import argparse
import copy
import datetime
import glob
import os
import platform
import subprocess
import threading
import time

from typing import List

import numpy as np

from cscore_utils.CSCoreCamera import CSCoreCamera
from cscore_utils.usbCameraUtils import generateCameraParameters

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2

parser = argparse.ArgumentParser()
parser.add_argument('-d', dest='image_directory', action="store", type=str, help='Use images in given directory')
parser.add_argument('-s', dest='save_images', action="store", type=bool, default=False, help='Save images for reprocessing later (Default: false)')
parser.add_argument('-c', dest='cam_name', action="store", type=str, default="OV2311_1", help='Camera to calibrate (Default: OV2311_1)')
parser.add_argument('-r', dest='rate', action="store", type=float, default=5, help='Camera frame captue rate per second. (Default: 5)')

args = parser.parse_args()

cap_props = [x for x in dir(cv2) if x.startswith('CAP_PROP_')]


class CameraCalibrate:
    _all_charuco_corners: List[np.ndarray] = []
    _all_charuco_ids: List[np.ndarray] = []
    _imsize = None

    def __init__(self, device_name) -> None:
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        # Squares X, Squares Y, Squares L, Markers  L
        # self._charuco_board = cv2.aruco.CharucoBoard((12, 9), 0.030, 0.023, self._aruco_dict)
        self._charuco_board = cv2.aruco.CharucoBoard((12, 9), 0.021, 0.016, self._aruco_dict)
        self._device_name = device_name
        self._filename = "{}.csv".format(device_name)
        self._detector = cv2.aruco.CharucoDetector(self._charuco_board)

    def process_frame(self, image: cv2.Mat) -> bool:
        # Get image size
        if self._imsize is None:
            self._imsize = (image.shape[0], image.shape[1])

        # Detect tags
        (charucoCorners, charucoIds, makerCorners, markerIds) = self._detector.detectBoard(image)

        if charucoIds is None:
            print("No Charuco IDs found!")
            return False

        numIds = len(charucoIds)
        numMarkers = len(markerIds)
        save = True if numIds > 20 and numMarkers > 20 else False
        print("{} charucoIds and {} markers found.".format(numIds, numMarkers))
        if save:
            cv2.aruco.drawDetectedMarkers(image, makerCorners)

            cv2.aruco.drawDetectedCornersCharuco(image, charucoCorners, charucoIds)

            # Save corners
            if save:
                self._all_charuco_corners.append(charucoCorners)
                self._all_charuco_ids.append(charucoIds)
                print("Saved calibration frame")
            return True
        else:
            print("Frame did not have enough ids. Skipping...")
            return False

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
    cam_name = []
    if args.cam_name:
        cam_name = args.cam_name
    elif platform.system() == 'Linux':
        cam_name = subprocess.check_output('v4l2-ctl --list-devices | grep "OV2311"')

    camera_params = generateCameraParameters(cam_name)
    camera_calibration = CameraCalibrate(cam_name)
    if platform.system() == 'Linux':
        os.system("pkill -f UsbHost")
        os.system('./../coprocessors/startup/initCameraSettings.sh')
    camera = CSCoreCamera(camera_params, True)

    maxSamples = 30
    sampleCount = 0
    frameCapture = 0
    while sampleCount < maxSamples:
        frameCapture += 1
        print("Frame {:05d} - ({}/{}): Capturing Frame...".format(frameCapture, sampleCount, maxSamples))
        timestamp, rawFrame = camera.getFrame()

        frame = copy.copy(rawFrame)

        retVal = camera_calibration.process_frame(frame)
        if retVal and args.save_images:
            if platform.system() == 'Windows':
                dir = os.path.join(os.getcwd(), "resources\\calibration_images")
            else:
                dir = os.path.join(os.getcwd(), "resources/calibration_images")
            if not os.path.isdir(dir):
                os.mkdir(dir)

            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            if platform.system() == 'Windows':
                filename = "{}\\{}\\{}-{}.jpg".format(dir, cam_name, timestamp, cam_name)
                cv2.imwrite(filename, rawFrame)
            else:
                filename = "{}/{}/{}-{}.jpg".format(dir, cam_name, timestamp, cam_name)
                cv2.imwrite(filename, rawFrame)
            print("Saved image as {}".format(filename))

        sampleCount += 1 if retVal else 0
        # Wait 5 seconds between frame captures
        time.sleep(args.rate)

    camera_calibration.finish()


if __name__ == '__main__':
    main()
