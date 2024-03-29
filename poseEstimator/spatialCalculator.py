import math
import numpy as np
from wpimath.geometry import Transform3d, CoordinateSystem, Translation3d, Rotation3d

from common.mathUtils import euler_from_quaternion


class HostSpatialsCalc:
    # We need device object to get calibration data
    def __init__(self, camera_params, tag_dictionary, log):
        # calibData = device.readCalibration()
        # Required information for calculating spatial coordinates on the host
        self.monoHFOV = np.deg2rad(camera_params['hfov'])
        self.monoVFOV = np.deg2rad(camera_params['vfov'])
        self.mountAngle = camera_params['mount_angle_radians']
        self.tagDictionary = tag_dictionary
        self.log = log

        # Values
        self.DELTA = 5
        self.THRESH_LOW = 200  # 20cm
        self.THRESH_HIGH = 30000  # 30m

    def setLowerThreshold(self, threshold_low):
        self.THRESH_LOW = threshold_low

    def setUpperThreshold(self, threshold_low):
        self.THRESH_HIGH = threshold_low

    def setDeltaRoi(self, delta):
        self.DELTA = delta

    def _check_input(self, roi, frame):  # Check if input is ROI or point. If point, convert to ROI
        if len(roi) == 4: return roi
        if len(roi) != 2: raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
        # Limit the point so ROI won't be outside the frame
        self.DELTA = 5  # Take 10x10 depth pixels around point for depth averaging
        x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
        y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
        return (x - self.DELTA, y - self.DELTA, x + self.DELTA, y + self.DELTA)

    def _calc_h_angle(self, frame, offset):
        return math.atan(math.tan(self.monoHFOV / 2.0) * offset / (frame.shape[1] / 2.0))

    def _calc_v_angle(self, frame, offset):
        return math.atan(math.tan(self.monoVFOV / 2.0) * offset / (frame.shape[0] / 2.0))

    # roi has to be list of ints
    def calc_spatials(self, depthFrame, tag, roi, robotAngles, averaging_method=np.mean):
        tagPose = self.tagDictionary['tags'][tag.getId() - 1]['pose']['translation']
        tagEuler = self.tagDictionary['tags'][tag.getId() - 1]['pose']['rotation']['euler']
        # roi = self._check_input(roi, depthFrame)  # If point was passed, convert it to ROI
        xmin, ymin, xmax, ymax = roi

        # Calculate the average depth in the ROI.
        depthROI = depthFrame[ymin:ymax, xmin:xmax]
        inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)

        averageDepth = averaging_method(depthROI[inRange])

        centroid = {  # Get centroid of the ROI
            'x': tag.getCenter().x,
            'y': tag.getCenter().y
        }

        midW = int(depthFrame.shape[1] / 2.0)  # middle of the depth img width
        midH = int(depthFrame.shape[0] / 2.0)  # middle of the depth img height
        bb_x_pos = centroid['x'] - midW
        bb_y_pos = centroid['y'] - midH

        angle_x = self._calc_h_angle(depthFrame, bb_x_pos)
        angle_y = -self._calc_v_angle(depthFrame, bb_y_pos)

        spatials = {
            'z': averageDepth / 1000.0,
            'x': averageDepth * math.tan(angle_x) / 1000.0,
            'y': averageDepth * math.tan(angle_y) / 1000.0
        }
        camera_angle = self.mountAngle if robotAngles['pitch'] is None else robotAngles['pitch']
        xy_target_distance = math.cos(camera_angle + angle_y) * spatials['z']

        camera_yaw = 0 if robotAngles['yaw'] is None else robotAngles['yaw']
        # tagTranslation = {
        #     # In WPILib coordinates
        #     'x': math.cos(angle_x + camera_yaw + (math.pi - tagEuler['yaw'])) * xy_target_distance,
        #     'y': -math.sin(angle_x + camera_yaw + (math.pi - tagEuler['yaw'])) * xy_target_distance,
        #     'z': math.sin(camera_angle + angle_y) * spatials['z'],
        #     # In Camera orientation
        #     'x_angle': math.degrees(angle_x),
        #     'y_angle': math.degrees(angle_y)
        # }
        tagTranslation = Translation3d(spatials['x'], spatials['y'], spatials['z'])
        tagRotation = Rotation3d(angle_x, angle_y, camera_yaw)

        wpiTransform = CoordinateSystem.convert(tagTranslation,
                                                CoordinateSystem.EDN(),
                                                CoordinateSystem.NWU())

        robotPose = tagPose.transformBy(Transform3d(wpiTransform, tagRotation))

        return robotPose, tagTranslation, spatials


def estimate_robot_pose_from_apriltag(tag, spatialData, camera_params, tagDictionary, frame_shape):
    tagPose = self.tagDictionary['tags'][tag.getId() - 1]['pose']['translation']

    horizontal_angle_radians = math.atan((tag.center[0] - (frame_shape[1] / 2.0)) / camera_params["hfl"])
    vertical_angle_radians = -math.atan((tag.center[1] - (frame_shape[0] / 2.0)) / camera_params["vfl"])
    horizontal_angle_degrees = math.degrees(horizontal_angle_radians)
    vertical_angle_degrees = math.degrees(vertical_angle_radians)

    xy_target_distance = math.cos(camera_params['mount_angle_radians'] + vertical_angle_radians) * spatialData['z']

    # Calculate the translation from the camera to the tag, in field coordinates
    tag_translation = {
        'x': math.cos(horizontal_angle_radians) * xy_target_distance,
        'y': -math.sin(horizontal_angle_radians) * xy_target_distance,
        'z': math.sin(camera_params['mount_angle_radians'] + vertical_angle_radians) * spatialData['z'],
        'x_angle': horizontal_angle_degrees,
        'y_angle': vertical_angle_degrees
    }

    robotPose = {
        'x': tagPose['x'] - tag_translation['x'],
        'y': tagPose['y'] - tag_translation['y'],
        'z': tagPose['z'] - tag_translation['z']
    }

    return robotPose, tag_translation


def estimate_robot_pose_with_solvePnP(tag, tagInfo, tagPose, camera_params, robotAngles):
    # rvec = np.squeeze(tag.pose_R[0], axis=None)
    # rvec_matrix = cv2.Rodrigues(rvec)[0]
    # proj_matrix = np.hstack((rvec_matrix, tag.pose_t))
    # euler_angles = cv2.decomposeProjectionMatrix(proj_matrix)[6]

    camera_pitch = camera_params['mount_angle_radians'] if robotAngles['pitch'] is None else robotAngles['pitch']
    xy_target_distance = math.cos(camera_pitch + math.radians(tagInfo['YAngle'])) * tagPose['y']

    camera_yaw = 0 if robotAngles['yaw'] is None else robotAngles['yaw']
    x_translation = math.cos(math.radians(tagInfo['XAngle']) + camera_yaw) * xy_target_distance
    y_translation = -math.sin(math.radians(tagInfo['XAngle']) + camera_yaw) * xy_target_distance

    robotPose = {
        'x': tagPose['x'] - x_translation,
        'y': tagPose['y'] - y_translation
    }

    return robotPose
