import argparse
import copy
import os
import platform
import time

import cv2
import logging
import math
import numpy as np
from os.path import basename
import socket
import sys

from wpimath.geometry import Translation3d, Rotation3d, Pose3d, Quaternion, CoordinateSystem, Transform3d

from common.cvUtils import TargetDrawer, drawStats
from cscore_utils.CSCoreCamera import CSCoreCamera

from ntcore import NetworkTableInstance

from aprilTags.apriltagDetector import AprilTagDetector
from common import constants
from common import utils
from cscore_utils.CSCoreServer import CSCoreServer
from aprilTags.tag_dictionary import TAG_DICTIONARY
from cscore_utils.usbCameraUtils import generateCameraParameters

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
parser.add_argument('-imu', dest='imu', action="store_true", default=False, help='Use external IMU')
parser.add_argument('-r', dest='record_video', action="store_true", default=False, help='Record video data')
parser.add_argument('-dic', dest='tag_dictionary', action="store", type=str, default='test', help='Set Tag Dictionary')
parser.add_argument('-p', dest='position', action="store", type=str, help='Enforce Device Position', choices=['Left_Localizers', 'Right_Localizers'])
parser.add_argument('-port', dest='ports', action="store", type=str, help='Set MJPEG Server Ports (default: 5800, 5801)', default="5800, 5801")
parser.add_argument('-dev', dest='dev', action="store", type=str, help='Set Device Name (default: OV2311_1)', default="OV2311_1")


args = parser.parse_args()

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.INFO)


class AprilTagsUSBHost:
    initialized = False

    def __init__(self):
        self.camera = None
        self.NT_Instance = self.init_networktables()
        if args.performance_test:
            disabledStdOut = logging.StreamHandler(stream=None)
            log.addHandler(disabledStdOut)

        log.info("Starting AprilTag Detector")

        self.DEBUG_VIDEO_OUTPUT = args.test
        self.ENABLE_SOLVEPNP_VISUALIZATION = True
        self.ENABL_LINUX_OPTIMIZATION = False

        self.valid_cameras = None
        if args.position is not None:
            self.valid_cameras = constants.CAMERAS[args.p]

        self.fps = utils.FPSHandler()

        self.tag_dictionary = TAG_DICTIONARY
        self.valid_tags = [t['ID'] for t in self.tag_dictionary['tags']]

        self.latency = np.array([0])

        self.device_name = args.dev
        self.mjpeg_server_ports = list(map(int, args.ports.split(",")))

        # camera setup
        self.camera_params = generateCameraParameters(self.device_name)
        self.cameraSetup()
        
        self.nt_drivetrain_tab = self.NT_Instance.getTable("Swerve")
        self.nt_apriltag_tab = self.NT_Instance.getTable(self.camera_params["nt_name"])
        self.nt_subs = {
            'Pitch': self.nt_drivetrain_tab.getDoubleTopic("Pitch").subscribe(0),
            'Roll': self.nt_drivetrain_tab.getDoubleTopic("Roll").subscribe(0),
            'Yaw': self.nt_drivetrain_tab.getDoubleTopic("Yaw").subscribe(0),
            'camToRobotT3D': self.nt_apriltag_tab.getDoubleArrayTopic("camToRobotT3D").subscribe([0, 0, 0, 0, 0, 0]),
        }
        self.nt_pubs = {
            'tv': self.nt_apriltag_tab.getIntegerTopic("tv").publish(),
            'tid': self.nt_apriltag_tab.getIntegerArrayTopic("tid").publish(),
            'rPosX': self.nt_apriltag_tab.getDoubleArrayTopic("Robot Pose X").publish(),
            'rPosY': self.nt_apriltag_tab.getDoubleArrayTopic("Robot Pose Y").publish(),
            'rPosYaw': self.nt_apriltag_tab.getDoubleArrayTopic("Robot Pose Yaw").publish(),
            'tPosX': self.nt_apriltag_tab.getDoubleArrayTopic("Tag Pose X").publish(),
            'tPosY': self.nt_apriltag_tab.getDoubleArrayTopic("Tag Pose Y").publish(),
            'latency': self.nt_apriltag_tab.getDoubleTopic("latency").publish(),
            'botpose': self.nt_apriltag_tab.getDoubleArrayTopic("botpose").publish(),
        }
        self.camera_stream = CSCoreServer(self.camera,
                                          ports=self.mjpeg_server_ports,
                                          width=self.camera_params["width"],
                                          height=self.camera_params["height"],
                                          fps=self.camera_params["fps"])

        self.detector = AprilTagDetector(args, self.camera_params)

        self.gyro = None

        self.stats = {
            'numTags': 0,
            'depthAI': {
                'x_pos': [0],
                'y_pos': [0],
                'z_pos': [0],
            },
            'solvePnP': {
                'x_pos': [0],
                'y_pos': [0],
                'z_pos': [0],
            }
        }

        self.run_thread = None

        self.lastTimestamp = 0

        self.camera_settings = None
        if self.DEBUG_VIDEO_OUTPUT:
            from PyQt5 import QtWidgets
            from designer.debugWindow import DebugWindow
            self.app = QtWidgets.QApplication(sys.argv)
            self.testGui = DebugWindow(self.gyro, self.ENABLE_SOLVEPNP_VISUALIZATION)
            self.camera_settings = self.testGui.getCameraSettings()
            # app.exec_()

    def run(self):
        log.debug("Setup complete, parsing frames...")
        try:
            self.detect_apriltags()
        except KeyboardInterrupt as e:
            log.info("Keyboard Interrupt")
            if self.run_thread is not None:
                self.run_thread.join()

    def detect_apriltags(self):
        frame = np.zeros(shape=(self.camera_params["height"], self.camera_params["width"], 1), dtype=np.uint8)
        while True:
            if self.camera is not None:
                if platform.system() == 'Linux' and self.ENABL_LINUX_OPTIMIZATION:
                    retval, frame = self.camera.read()
                    timestamp = time.time_ns()
                    if not retval:
                        print("Capture session failed, restarting")
                        self.camera.release()
                        self.camera = None  # Force reconnect
                        time.sleep(2)
                        continue
                    #frame = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
                    # log.info("Frame Shape: {}".format(frame.shape))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

                    self.process_results(frame, timestamp)
                else:
                    timestamp, frame = self.camera.getFrame()

                self.process_results(frame, timestamp)
            else:
                log.warning("Camera not found! Restarting camera init...")
                self.cameraSetup()
            
    def process_results(self, frame, timestamp):
        if self.gyro is not None:
            try:
                robotAngles = {
                    'pitch': math.radians(self.gyro.get('pitch')),
                    'yaw': math.radians(self.gyro.get('yaw'))
                }
                if self.DEBUG_VIDEO_OUTPUT:
                    self.testGui.updateYawValue(math.degrees(-robotAngles['yaw']))
                    self.testGui.updatePitchValue(math.degrees(robotAngles['pitch']))
            except Exception as e:
                # log.error("Could not grab gyro values")
                pass
        else:
            robotAngles = {
                'pitch': math.radians(self.nt_subs['Pitch'].get()),
                'roll': math.radians(self.nt_subs['Roll'].get()),
                'yaw': math.radians(self.nt_subs['Yaw'].get())
            }

        cameraToRobotV = self.nt_subs['camToRobotT3D'].get()
        cameraToRobotTransform = Transform3d(Translation3d(cameraToRobotV[0], cameraToRobotV[1], cameraToRobotV[2]),
                                             Rotation3d(cameraToRobotV[3], cameraToRobotV[4], cameraToRobotV[5]))

        monoFrame = frame
        fps = self.fps
        detector = self.detector

        fps.nextIter()
        if self.DEBUG_VIDEO_OUTPUT:
            # print("\033[2J", end='')
            pass

        if self.DEBUG_VIDEO_OUTPUT:
            self.ENABLE_SOLVEPNP_VISUALIZATION = self.testGui.getSolvePnpEnabled()

            if self.testGui.getPauseResumeState():
                monoFrame = self.lastMonoFrame

        tags = detector.detect(monoFrame)

        detectedTags = []
        robot_pose_x = []
        robot_pose_y = []
        robot_pose_z = []
        robot_pose_yaw = []

        tag_id = []
        tag_pose_x = []
        tag_pose_y = []
        tag_pose_z = []
        if len(tags) > 0:
            for tag in tags:
                if self.DEBUG_VIDEO_OUTPUT:
                    if tag.getId() not in self.testGui.getTagFilter():
                        continue
                if tag.getId() not in self.valid_tags:
                    log.warning("Tag ID {} found, but not defined".format(tag.getId()))
                    continue
                elif tag.getDecisionMargin() < 35:
                    log.warning("Tag {} found, but not valid".format(tag.getId()))
                    continue
                tagCorners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]
                xPixels = [p.x for p in tagCorners]
                yPixels = [p.y for p in tagCorners]
                topLeftXY = (int(min(xPixels)), int(min(yPixels)))
                bottomRightXY = (int(max(xPixels)), int(max(yPixels)))

                tagValues = self.tag_dictionary['tags'][tag.getId() - 1]['pose']
                tagTranslation = tagValues['translation']
                tagRotation = tagValues['rotation']['quaternion']
                tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
                tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
                tagPose = Pose3d(tagT3D, tagR3D)
                tagTranslation = detector.estimatePose(tag)

                wpiTransform = CoordinateSystem.convert(tagTranslation,
                                                        CoordinateSystem.EDN(),
                                                        CoordinateSystem.NWU())

                estimatedRobotPose = tagPose.transformBy(Transform3d(wpiTransform.translation(), tagTranslation.rotation())) \
                                            .transformBy(cameraToRobotTransform)

                tagInfo = {
                    "tag": tag,
                    "corners": tagCorners,
                    "topLeftXY": topLeftXY,
                    "bottomRightXY": bottomRightXY,
                    "tagPose": tagPose,
                    "tagTranslation": tagTranslation,
                    "estimatedRobotPose": estimatedRobotPose
                }

                detectedTags.append(tagInfo)

        detectedTags = sorted(detectedTags, key=lambda d: d['tag'].getDecisionMargin(), reverse=True)
        for detectedTag in detectedTags:
            robot_pose_x.append(detectedTag["estimatedRobotPose"].translation().x)
            robot_pose_y.append(detectedTag["estimatedRobotPose"].translation().y)
            robot_pose_yaw.append(detectedTag["estimatedRobotPose"].rotation().y_degrees)
            tag_pose_x.append(detectedTag["tagPose"].translation().x)
            tag_pose_y.append(detectedTag["tagPose"].translation().y)
            tag_pose_z.append(detectedTag["tagPose"].translation().z)
            tag_id.append(detectedTag["tag"].getId())

        fpsValue = self.fps.fps()
        if self.lastTimestamp != 0:
            latencyMs = (timestamp - self.lastTimestamp) / 1000000.0
            self.latency = np.append(self.latency, latencyMs)
        avgLatency = np.average(self.latency) if len(self.latency) < 100 else np.average(self.latency[-100:])
        latencyStd = np.std(self.latency) if len(self.latency) < 100 else np.std(self.latency[-100:])
        self.lastTimestamp = timestamp

        # Merge AprilTag measurements to estimate
        # Strategy 1: Average all tags
        # botPose = [np.average(robot_pose_x), np.average(robot_pose_y), np.average(robot_pose_z)]
        # Strategy 2: Choose best tag by decision Margin
        # if len(tag_id) > 0:
        #     botPose = [robot_pose_x[0], robot_pose_y[0], robot_pose_z[0]]
        #     log.info("Estimated Pose X: {:.2f}\tY: {:.2f}\tZ: {:.2f}".format(botPose[0], botPose[1], botPose[2]))
        # else:
        #     botPose = []

        # Strategy 3: Remove tags from list if they are outside tolerance
        x_mean = np.mean(robot_pose_x)
        y_mean = np.mean(robot_pose_y)
        yaw_mean = np.mean(robot_pose_yaw)
        x_std = np.std(robot_pose_x)
        y_std = np.std(robot_pose_x)
        yaw_std = np.std(robot_pose_yaw)
        std_multiplier = 5
        rm_idx = []
        if len(tag_id) > 1:
            for pose_idx in range(len(tag_id)):
                if math.fabs(robot_pose_x[pose_idx] - x_mean) > x_std * std_multiplier:
                    rm_idx.append(pose_idx)
                if math.fabs(robot_pose_y[pose_idx] - y_mean) > y_std * std_multiplier:
                    rm_idx.append(pose_idx)
                if math.fabs(robot_pose_yaw[pose_idx] - yaw_mean) > yaw_std * std_multiplier:
                    rm_idx.append(pose_idx)

        rm_idx = np.unique(rm_idx)
        tag_id = [i for j, i in enumerate(tag_id) if j not in rm_idx]
        robot_pose_x = [i for j, i in enumerate(robot_pose_x) if j not in rm_idx]
        robot_pose_y = [i for j, i in enumerate(robot_pose_y) if j not in rm_idx]
        robot_pose_yaw = [i for j, i in enumerate(robot_pose_yaw) if j not in rm_idx]

        botPose = [np.average(robot_pose_x), np.average(robot_pose_y), 0,
                   0, 0, np.average(robot_pose_yaw),
                   self.latency[-1]]

        log.info("Std dev X: {:.2f}\tY: {:.2f}".format(x_std, y_std))
        self.nt_pubs["tv"].set(1 if len(tag_id) > 0 else 0)
        self.nt_pubs["tid"].set(tag_id)
        self.nt_pubs["rPosX"].set(robot_pose_x)
        self.nt_pubs["rPosY"].set(robot_pose_y)
        self.nt_pubs["rPosYaw"].set(robot_pose_yaw)
        self.nt_pubs["tPosX"].set(tag_pose_x)
        self.nt_pubs["tPosY"].set(tag_pose_y)
        # self.nt_apriltag_tab.putNumber("Heading Pose", 0 if robotAngles['yaw'] is None else robotAngles['yaw'])
        self.nt_pubs["latency"].set(self.latency[-1])
        self.nt_pubs["botpose"].set(botPose)

        self.stats = {
            'numTags': len(detectedTags),
            'depthAI': {
                'robot_pose_x': robot_pose_x,
                'robot_pose_y': robot_pose_y,
                'robot_pose_z': robot_pose_z,
            }
        }

        for detectedTag in detectedTags:
            points = np.array([(int(p.x), int(p.y)) for p in detectedTag["corners"]])

            targetDrawer = TargetDrawer(points)
            targetDrawer.drawTargetLines(monoFrame)

            statText = [
                "FPS: {:.2f}".format(fpsValue),
                "Latency: {:.2f}ms".format(avgLatency)
            ]
            drawStats(monoFrame, statText)

            if self.ENABLE_SOLVEPNP_VISUALIZATION:
                targetDrawer.drawTargetBox(monoFrame, self.camera_params['iMatrix'], detectedTag["tagTranslation"])

            if self.DEBUG_VIDEO_OUTPUT:
                targetDrawer.addText(monoFrame, "tag_id: {}".format(detectedTag['tag'].getId()))
                targetDrawer.addText(monoFrame, "x: {:.2f}".format(detectedTag["tagTranslation"].translation().x))
                targetDrawer.addText(monoFrame, "y: {:.2f}".format(detectedTag["tagTranslation"].translation().y))
                targetDrawer.addText(monoFrame, "z: {:.2f}".format(detectedTag["tagTranslation"].translation().z))
                targetDrawer.addText(monoFrame, "pitch: {:.2f}".format(detectedTag["tagTranslation"].rotation().x_degrees))
                targetDrawer.addText(monoFrame, "roll: {:.2f}".format(detectedTag["tagTranslation"].rotation().z_degrees))
                targetDrawer.addText(monoFrame, "yaw: {:.2f}".format(detectedTag["tagTranslation"].rotation().y_degrees))

        if self.DEBUG_VIDEO_OUTPUT:
            # cv2.imshow(pipeline_info["monoRightQueue"], frameRight)
            # cv2.imshow(pipeline_info["depthQueue"], depthFrameColor)
            self.testGui.updateTagIds(tag_id)
            if len(detectedTags) > 0:
                self.testGui.updateStatsValue(self.stats)
            self.testGui.updateFrames(monoFrame, monoFrame)
        # if not self.DEBUG_VIDEO_OUTPUT:
        #     print('FPS: {:.2f}\tLatency: {:.2f} ms\tStd: {:.2f}'.format(fpsValue, avgLatency, latencyStd))
        #     print('DepthAI')
        #     print('Tags: {}'.format(tag_id))
        #     print("         Avg.: {:.6f}\t{:.6f}\t{:.6f}".format(np.average(robot_pose_x), np.average(robot_pose_y), np.average(robot_pose_z)))
        #     print("         Std.: {:.6f}\t{:.6f}\t{:.6f}".format(np.average(robot_pose_x), np.average(robot_pose_y), np.average(robot_pose_z)))
            # print("\033[2J", end='')

        self.lastMonoFrame = copy.copy(monoFrame)
        self.camera_stream.setFrame(monoFrame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            raise StopIteration()
        elif key == ord(' '):
            if self.gyro is not None:
                self.gyro.resetAll()

        if self.DEBUG_VIDEO_OUTPUT:
            if not self.testGui.isVisible():
                pass

    def init_networktables(self):
        NT_Instance = NetworkTableInstance.getDefault()
        identity = f"{basename(__file__)}-{os.getpid()}"
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
        time.sleep(1)
        while not NT_Instance.isConnected():
            log.debug("Could not connect to team client. Trying other addresses...")
            NT_Instance.setServer(secondaryIps[tries])

            time.sleep(1)
            if NT_Instance.isConnected():
                log.info("Found NT Server at {}".format(secondaryIps[tries]))
                break
            tries += 1
            if tries >= len(secondaryIps):
                log.error("Could not connect to NetworkTables...")
                break

        return NT_Instance

    def cameraSetup(self):
        # if platform.system() == 'Linux' and self.ENABLE_LINUX_OPTIMIZATION:
        #     self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        #     self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_params["height"])
        #     self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_params["width"])
        #     self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        #     self.camera.set(cv2.CAP_PROP_FPS, self.camera_params["fps"])
        #     self.camera.set(cv2.CAP_PROP_GAIN, self.camera_params["gain"])
        #     self.camera.set(cv2.CAP_PROP_EXPOSURE, -11)
        #     self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
        #     self.camera.set(cv2.CAP_PROP_SHARPNESS, 0)
        #     self.camera = cv2.VideoCapture("v4l2src device=/dev/video" + str(0) +
        #                                   " extra_controls=\"c,exposure_auto=" + str(self.camera_params["exposure_auto"]) +
        #                                   ",exposure_absolute=" + str(self.camera_params["exposure"]) +
        #                                   ",gain=" + str(self.camera_params["gain"]) +
        #                                   ",sharpness=0,brightness=0\" ! image/jpeg,format=MJPG,width=" + str(self.camera_params["width"]) +
        #                                   ",height=" + str(self.camera_params["height"]) + " ! jpegdec ! video/x-raw ! appsink drop=1", cv2.CAP_GSTREAMER)
        # else:
        self.camera = CSCoreCamera(self.camera_params, True)
    

if __name__ == '__main__':
    log.info("Starting AprilTagsUSBHost")
    AprilTagsUSBHost().run()
