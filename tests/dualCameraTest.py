import argparse
import math
from os.path import basename
import socket

import cv2
import depthai as dai
import numpy as np
import robotpy_apriltag
import wpimath.geometry
from ntcore import NetworkTableInstance
from wpimath import geometry

from aprilTags.apriltagDetector import AprilTagDetector
from aprilTags.tag_dictionary import TAG_DICTIONARY
from common import utils, constants
from pipelines import apriltags_pipeline


parser = argparse.ArgumentParser()
parser.add_argument('-d', dest='debug', action="store_true", default=False, help='Start in Debug Mode')
parser.add_argument('-t', dest='test', action="store_true", default=False, help='Start in Test Mode')
parser.add_argument('-pt', dest='performance_test', action="store_true", default=False, help='Set Performance Test Mode')
parser.add_argument('-f', dest='family', action="store", type=str, default='tag16h5',
                    help='Tag family (default: tag16h5)')
parser.add_argument('-nt', dest='nthreads', action="store", type=int, default=3,
                    help='nthreads (default: 3)')
parser.add_argument('-qd', dest='quad_decimate', action="store", type=float, default=2.0,
                    help='quad_decimate (default: 2)')
parser.add_argument('-qs', dest='quad_sigma', action="store", type=float, default=0.0,
                    help='quad_sigma (default: 0.0)')
parser.add_argument('-re', dest='refine_edges', action="store", type=float, default=1.0,
                    help='refine_edges (default: 1.0)')
parser.add_argument('-ds', dest='decode_sharpening', action="store", type=float, default=0.25,
                    help='decode_sharpening (default: 0.25)')
parser.add_argument('-dd', dest='detector_debug', action="store", type=int, default=0,
                    help='AprilTag Detector debug mode (default: 0)')

parser.add_argument('-pnp', dest='apriltag_pose', action="store_true", default=False,
                    help='Enable pupil_apriltags Detector Pose Estimation')
parser.add_argument('-imu', dest='imu', action="store_true", default=False, help='Use external IMU')
parser.add_argument('-r', dest='record_video', action="store_true", default=False, help='Record video data')

parser.add_argument('-dic', dest='tag_dictionary', action="store", type=str, default='test', help='Set Tag Dictionary')

args = parser.parse_args()


def main():
    pipeline, pipeline_info = apriltags_pipeline.create_dual_mono_apriltag_pipeline()
    # pipeline = apriltags_pipeline.create_mono_apriltag_pipeline()

    fpsL = utils.FPSHandler()
    fpsR = utils.FPSHandler()
    latency = np.array([])

    NT_Instance = init_networktables()

    tag_dictionary = TAG_DICTIONARY
    valid_tags = [t['ID'] for t in tag_dictionary['tags']]
    robotToCamera = geometry.Transform3d()

    with dai.Device(pipeline) as device:
        leftQueue = device.getOutputQueue(name="monoLeft", maxSize=1, blocking=False)
        rightQueue = device.getOutputQueue(name="monoRight", maxSize=1, blocking=False)

        deviceID = device.getMxId()
        eepromData = device.readCalibration().getEepromData()
        iMatrixL = eepromData.cameraData.get(dai.CameraBoardSocket.LEFT).intrinsicMatrix
        iMatrixR = eepromData.cameraData.get(dai.CameraBoardSocket.RIGHT).intrinsicMatrix

        if deviceID in constants.CAMERAS:
            productName = constants.CAMERAS[deviceID]['name']
        else:
            productName = eepromData.productName

            if len(productName) == 0:
                boardName = eepromData.boardName
                if boardName == 'BW1098OBC':
                    productName = 'OAK-D'
                else:
                    productName = 'OAK-D'

        camera_paramsL = {
            "hfov": constants.CAMERA_PARAMS[productName]["mono"]["hfov"],
            "vfov": constants.CAMERA_PARAMS[productName]["mono"]["vfov"],
            "iMatrix": np.array(iMatrixL).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrixL[0][0], iMatrixL[1][1], iMatrixL[0][2], iMatrixL[1][2]),
        }
        camera_paramsR = {
            "hfov": constants.CAMERA_PARAMS[productName]["mono"]["hfov"],
            "vfov": constants.CAMERA_PARAMS[productName]["mono"]["vfov"],
            "iMatrix": np.array(iMatrixR).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrixR[0][0], iMatrixR[1][1], iMatrixR[0][2], iMatrixR[1][2]),
        }

        detectorL = AprilTagDetector(args, camera_paramsL)
        detectorR = AprilTagDetector(args, camera_paramsR)

        cameras = [
            {
                'camera': 'leftMono',
                'camera_params': camera_paramsL,
                'queue': leftQueue,
                'detector': detectorL,
                'fps': fpsL
            },
            {
                'camera': 'rightMono',
                'camera_params': camera_paramsR,
                'queue': rightQueue,
                'detector': detectorR,
                'fps': fpsR
            },

        ]
        nt_depthai_tab = NT_Instance.getTable('fLocalizer')

        cameras = [cameras[1]]
        while True:
            for camera in cameras:
                robot_pos_x = []
                robot_pos_y = []
                robot_pos_z = []
                tag_pos_x = []
                tag_pos_y = []
                tag_pos_z = []
                pose_id = []
                try:
                    inFrame = camera['queue'].get()  # blocking call, will wait until a new data has arrived
                except Exception as e:
                    continue
                inMono = inFrame.getCvFrame()
                fps = camera['fps']
                detector = camera['detector']
                fps.nextIter()

                tags = detector.detect(inMono)
                fpsValue = fps.fps()
                latencyMs = (dai.Clock.now() - inFrame.getTimestamp()).total_seconds() * 1000.0
                latency = np.append(latency, latencyMs)
                avgLatency = np.average(latency) if len(latency) < 100 else np.average(latency[-100:])
                latencyStd = np.std(latency) if len(latency) < 100 else np.std(latency[-100:])

                if len(tags) > 0:
                    for tag in tags:
                        if tag.getId() not in range(1, 9):
                            continue

                        tagCorners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]
                        tagTransform = detector.estimatePose(tag)
                        tagValues = tag_dictionary['tags'][tag.getId() - 1]['pose']
                        tagTranslation = tagValues['translation']
                        tagRotation = tagValues['rotation']['quaternion']
                        tagT3D = geometry.Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
                        tagR3D = geometry.Rotation3d(geometry.Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
                        tagPose = geometry.Pose3d(tagT3D, tagR3D)

                        # tagEstimatePose = geometry.Pose3d(tagTransform.translation(), tagTransform.rotation())
                        # tagPoseTranslation = geometry.Transform3d(tagPose.translation(), tagPose.rotation())
                        robotPose = tagPose.transformBy(tagTransform.inverse()) \
                                           .transformBy(robotToCamera)

                        pose_id.append(tag.getId())
                        robot_pos_x.append(robotPose.translation().x)
                        robot_pos_y.append(robotPose.translation().y)
                        robot_pos_z.append(robotPose.translation().z)
                        tag_pos_x.append(tagPose.translation().x)
                        tag_pos_y.append(tagPose.translation().y)
                        tag_pos_z.append(tagPose.translation().z)

                        points = np.array([(int(p.x), int(p.y)) for p in tagCorners])
                        # Shift points since this is a snapshot
                        cv2.polylines(inMono, [points], True, (120, 120, 120), 3)

                        r_vec = np.array([tagTransform.rotation().x,
                                          tagTransform.rotation().y,
                                          tagTransform.rotation().z])
                        t_vec = np.array([tagTransform.translation().x,
                                          tagTransform.translation().y,
                                          tagTransform.translation().z])
                        ipoints, _ = cv2.projectPoints(constants.OPOINTS,
                                                       r_vec,
                                                       t_vec,
                                                       camera['camera_params']['iMatrix'],
                                                       np.zeros(5))

                        ipoints = np.round(ipoints).astype(int)

                        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

                        for i, j in constants.EDGES:
                            cv2.line(inMono, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

                        textX = min(points[:, 0])
                        textY = min(points[:, 1]) + 20
                        cv2.putText(inMono, "tag_id: {}".format(tag.getId()),
                                    (textX, textY), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))

                cv2.circle(inMono, (int(inMono.shape[1] / 2), int(inMono.shape[0] / 2)), 1, (255, 255, 255),
                           1)
                cv2.putText(inMono, "FPS: {:.2f}".format(fpsValue), (0, 24), cv2.FONT_HERSHEY_TRIPLEX, 1,
                            (255, 255, 255))
                cv2.putText(inMono, "Latency: {:.2f}ms".format(avgLatency), (0, 50), cv2.FONT_HERSHEY_TRIPLEX, 1,
                            (255, 255, 255))
                cv2.imshow(camera['camera'], inMono)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

                if camera['camera'] == 'rightMono':
                    botPose = [np.average(robot_pos_x), np.average(robot_pos_y), np.average(robot_pos_z)]
                    nt_depthai_tab.putNumberArray("tid", pose_id)
                    nt_depthai_tab.putNumberArray("Robot Pose X", robot_pos_x)
                    nt_depthai_tab.putNumberArray("Robot Pose Y", robot_pos_y)
                    nt_depthai_tab.putNumberArray("Robot Pose Z", robot_pos_z)
                    nt_depthai_tab.putNumberArray("Tag Pose X", tag_pos_x)
                    nt_depthai_tab.putNumberArray("Tag Pose Y", tag_pos_y)
                    nt_depthai_tab.putNumberArray("Tag Pose Z", tag_pos_z)
                    nt_depthai_tab.putNumberArray("botpose", botPose)


def _calc_h_angle(monoHFOV, frame, offset):
    return math.atan(math.tan(monoHFOV / 2.0) * offset / (frame.shape[1] / 2.0))


def _calc_v_angle(monoVFOV, frame, offset):
    return math.atan(math.tan(monoVFOV / 2.0) * offset / (frame.shape[0] / 2.0))


def init_networktables():
    NT_Instance = NetworkTableInstance.getDefault()
    identity = basename(__file__)
    NT_Instance.startClient4(identity)
    NT_Instance.setServer("10.42.1.2")
    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)

    secondaryIps = [
            ip
            # '10.42.1.2',
            # ('127.0.0.1', 57599),
            # ('localhost', 57823)
            # '10.0.0.2',
            # '192.168.100.25'
            # '192.168.100.25'
            # '172.22.64.1'
            # '169.254.254.200'
        ]
    tries = 0
    while not NT_Instance.isConnected():
        print("Could not connect to team client. Trying other addresses...")
        NT_Instance.setServer(secondaryIps[tries])
        tries = tries + 1

        if tries >= len(secondaryIps):
            break

    if NT_Instance.isConnected():
        print("NT Connected to {}".format(NT_Instance.getConnections()))
    else:
        print("Could not connect to NetworkTables. Restarting server...")

    return NT_Instance


if __name__ == '__main__':
    main()
