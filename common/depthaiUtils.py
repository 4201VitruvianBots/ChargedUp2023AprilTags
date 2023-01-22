import logging

import depthai as dai
import math

import numpy as np

from common import constants

log = logging.getLogger(__name__)


def generateCameraParameters(pipeline, pipeline_info, device_info):
    with dai.Device(pipeline, device_info) as device:
        log.info("USB SPEED: {}".format(device.getUsbSpeed()))
        if device.getUsbSpeed() not in [dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS]:
            log.warning("USB Speed is set to USB 2.0")

        deviceID = device.getMxId()
        eepromData = device.readCalibration().getEepromData()
        iMatrix = eepromData.cameraData.get(dai.CameraBoardSocket.RIGHT).intrinsicMatrix

        device_type = constants.CAMERAS['Forward_Localizers']
        if deviceID in constants.CAMERAS['Rear_Localizers']['ids'].keys():
            device_type = constants.CAMERAS['Rear_Localizers']

        deviceName = device_type['ids'][deviceID]
        hfov = constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"]
        vfov = constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"]

        camera_params = {
            "device_name": deviceName,
            "id": deviceID,
            "nt_name": device_type['nt_name'],
            "hfov": hfov,
            "vfov": vfov,
            "mount_angle_radians": math.radians(device_type['mount_angle'][0]),
            "iMatrix": np.array(iMatrix).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrix[0][0], iMatrix[1][1], iMatrix[0][2], iMatrix[1][2]),
            "hfl": pipeline_info["resolution_x"] / (2 * math.tan(math.radians(hfov) / 2)),
            "vfl": pipeline_info["resolution_y"] / (2 * math.tan(math.radians(vfov) / 2))
        }

        return camera_params
