import math
import os

import numpy as np

from common import constants
from aprilTags import readCSV


def generateCameraParameters(deviceID):
    deviceType = 'Left_Localizer'
    if deviceID in constants.CAMERAS['Right_Localizer']['ids'].keys():
        deviceType = 'Right_Localizer'
    device_params = constants.CAMERAS[deviceType]
    deviceName = device_params['ids'][deviceID]

    if os.path.isfile('{}.csv'.format(deviceID)):
        iMatrix = readCSV.readCsv(deviceID)
    else:
        iMatrix = constants.CAMERA_PARAMS[deviceName]["camera_matrix"][deviceID]

    if iMatrix.shape == (1, 9) or iMatrix.shape == (9, 1):
        iMatrix.reshape([3, 3])

    if iMatrix.shape != (3, 3):
        print("ERROR: camera iMatrix is not a 3x3 matrix!")

    hfov = constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"]
    vfov = constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"]

    camera_params = {
        "device_id": deviceID,
        "device_name": deviceName,
        "device_type": deviceType,
        "id": 0,
        "nt_name": device_params['nt_name'],
        "hfov": hfov,
        "vfov": vfov,
        # "height": constants.CAMERA_PARAMS[deviceName]["height"],
        # "width": constants.CAMERA_PARAMS[deviceName]["width"],
        "height": 1200,
        "width": 1600,
        "fps": constants.CAMERA_PARAMS[deviceName]["fps"],
        "pixelFormat": constants.CAMERA_PARAMS[deviceName]["pixelFormat"],
        "exposure_auto": 0,
        "exposure": 0.1,
        "gain": 200,
        "mount_angle_radians": math.radians(device_params['mount_angle'][0]),
        "iMatrix": np.array(iMatrix).reshape(3, 3),
        # fx, fy, cx, cy
        "intrinsicValues": (iMatrix[0][0], iMatrix[1][1], iMatrix[0][2], iMatrix[1][2]),
        "hfl": constants.CAMERA_PARAMS[deviceName]["width"] / (2 * math.tan(math.radians(hfov) / 2)),
        "vfl": constants.CAMERA_PARAMS[deviceName]["height"] / (2 * math.tan(math.radians(vfov) / 2))
    }

    return camera_params
