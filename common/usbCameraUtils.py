import math

import numpy as np

from common import constants


def generateOV2311CameraParameters(deviceID):
    device_type = 'Forward_Localizers'
    if deviceID in constants.CAMERAS['Rear_Localizers']['ids'].keys():
        device_type = 'Rear_Localizers'
    device_params = constants.CAMERAS[device_type]
    deviceName = device_params['ids'][deviceID]

    iMatrix = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

    hfov = constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"]
    vfov = constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"]

    camera_params = {
        "device_name": deviceName,
        "device_type": "OV2311",
        "id": 0,
        "nt_name": device_params['nt_name'],
        "hfov": hfov,
        "vfov": vfov,
        "height": constants.CAMERA_PARAMS[deviceName]["height"],
        "width": constants.CAMERA_PARAMS[deviceName]["width"],
        "fps": constants.CAMERA_PARAMS[deviceName]["fps"],
        "exposure_auto": 0,
        "exposure": 0.1,
        "gain": 100,
        "mount_angle_radians": math.radians(device_params['mount_angle'][0]),
        "iMatrix": np.array(iMatrix).reshape(3, 3),
        # fx, fy, cx, cy
        "intrinsicValues": (iMatrix[0][0], iMatrix[1][1], iMatrix[0][2], iMatrix[1][2]),
        "hfl": constants.CAMERA_PARAMS[deviceName]["width"] / (2 * math.tan(math.radians(hfov) / 2)),
        "vfl": constants.CAMERA_PARAMS[deviceName]["height"] / (2 * math.tan(math.radians(vfov) / 2))
    }

    return camera_params