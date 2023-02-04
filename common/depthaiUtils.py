import logging

import depthai as dai
import math

import numpy as np
import robotpy_apriltag

from aprilTags.apriltagDetector import AprilTagDetector
from common import constants, utils

log = logging.getLogger(__name__)


def generateCameraParameters(pipeline, pipeline_info, device_info):
    with dai.Device(pipeline, device_info) as device:
        log.info("USB SPEED: {}".format(device.getUsbSpeed()))
        if device.getUsbSpeed() not in [dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS]:
            log.warning("USB Speed is set to USB 2.0")

        deviceID = device.getMxId()
        eepromData = device.readCalibration().getEepromData()
        iMatrix = eepromData.cameraData.get(dai.CameraBoardSocket.RIGHT).intrinsicMatrix

        device_type = 'Forward_Localizers'
        if deviceID in constants.CAMERAS['Rear_Localizers']['ids'].keys():
            device_type = 'Rear_Localizers'
        device_params = constants.CAMERAS[device_type]

        deviceName = device_params['ids'][deviceID]
        hfov = constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"]
        vfov = constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"]

        camera_params = {
            "device_name": deviceName,
            "device_type": device_type,
            "id": deviceID,
            "nt_name": device_params['nt_name'],
            "hfov": hfov,
            "vfov": vfov,
            "mount_angle_radians": math.radians(device_params['mount_angle'][0]),
            "iMatrix": np.array(iMatrix).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrix[0][0], iMatrix[1][1], iMatrix[0][2], iMatrix[1][2]),
            "hfl": pipeline_info["resolution_x"] / (2 * math.tan(math.radians(hfov) / 2)),
            "vfl": pipeline_info["resolution_y"] / (2 * math.tan(math.radians(vfov) / 2))
        }

        return camera_params


def generateDualMonoCameraParameters(pipeline, pipeline_info, device_info):
    with dai.Device(pipeline, device_info) as device:
        log.info("USB SPEED: {}".format(device.getUsbSpeed()))
        if device.getUsbSpeed() not in [dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS]:
            log.warning("USB Speed is set to USB 2.0")

        deviceID = device.getMxId()
        eepromData = device.readCalibration().getEepromData()
        iMatrixL = eepromData.cameraData.get(dai.CameraBoardSocket.LEFT).intrinsicMatrix
        iMatrixR = eepromData.cameraData.get(dai.CameraBoardSocket.RIGHT).intrinsicMatrix

        device_type = 'Forward_Localizers'
        if deviceID in constants.CAMERAS['Rear_Localizers']['ids'].keys():
            device_type = 'Rear_Localizers'
        device_params = constants.CAMERAS[device_type]

        deviceName = device_params['ids'][deviceID]
        hfov = constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"]
        vfov = constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"]

        camera_paramsL = {
            "hfov": constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"],
            "vfov": constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"],
            "iMatrix": np.array(iMatrixL).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrixL[0][0], iMatrixL[1][1], iMatrixL[0][2], iMatrixL[1][2]),
        }
        camera_paramsR = {
            "hfov": constants.CAMERA_PARAMS[deviceName]["mono"]["hfov"],
            "vfov": constants.CAMERA_PARAMS[deviceName]["mono"]["vfov"],
            "iMatrix": np.array(iMatrixR).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrixR[0][0], iMatrixR[1][1], iMatrixR[0][2], iMatrixR[1][2]),
        }

        detectorIntrinsicsL = robotpy_apriltag.AprilTagPoseEstimator.Config(
            constants.TAG_SIZE_M,
            camera_paramsL['intrinsicValues'][0],
            camera_paramsL['intrinsicValues'][1],
            camera_paramsL['intrinsicValues'][2],
            camera_paramsL['intrinsicValues'][3])

        detectorIntrinsicsR = robotpy_apriltag.AprilTagPoseEstimator.Config(
            constants.TAG_SIZE_M,
            camera_paramsR['intrinsicValues'][0],
            camera_paramsR['intrinsicValues'][1],
            camera_paramsR['intrinsicValues'][2],
            camera_paramsR['intrinsicValues'][3])

        detectorPoseEstimatorL = robotpy_apriltag.AprilTagPoseEstimator(detectorIntrinsicsL)
        detectorPoseEstimatorR = robotpy_apriltag.AprilTagPoseEstimator(detectorIntrinsicsR)

        camera_params = {
            'cameras': [
                    {
                    'camera': 'leftMono',
                    'camera_params': camera_paramsL,
                    'queueName': pipeline_info['monoLeftQueue'],
                    'fps': utils.FPSHandler()
                },
                {
                    'camera': 'rightMono',
                    'camera_params': camera_paramsR,
                    'queueName': pipeline_info['monoRightQueue'],
                    'detector': detectorPoseEstimatorR,
                    'fps': utils.FPSHandler()
                }
            ],
            "device_name": deviceName,
            'nt_name': device_params['nt_name']
        }
        detectorL = camera_params['cameras'][0]
        detectorR = camera_params['cameras'][1]

        camera_params['cameras'][0]['detector'] = detectorL
        camera_params['cameras'][1]['detector'] = detectorR

        return camera_params
