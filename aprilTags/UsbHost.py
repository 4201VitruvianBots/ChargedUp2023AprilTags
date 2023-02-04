import argparse
import copy
import json
import os
import time

import cscore
from wpimath.geometry import Translation3d, Rotation3d, Pose3d, Quaternion, CoordinateSystem, Transform3d

from cscore_utils.CSCoreCamera import CSCoreCamera

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
import logging
import math
import numpy as np
from os.path import basename
import socket
import sys
import platform

from ntcore import NetworkTableInstance

from aprilTags.apriltagDetector import AprilTagDetector
from common import constants
from common import utils
from cscore_utils.CSCoreServer import CSCoreServer
from aprilTags.tag_dictionary import TAG_DICTIONARY
from cscore_utils.usbCameraUtils import generateOV2311CameraParameters

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

parser.add_argument('-p', dest='position', action="store", type=str, help='Enforce Device Position', choices=['Forward_Localizers', 'Rear_Localizers'])

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

        self.DISABLE_VIDEO_OUTPUT = args.test

        self.ENABLE_SOLVEPNP = True

        self.valid_cameras = None
        if args.position is not None:
            self.valid_cameras = constants.CAMERAS[args.p]

        self.fps = utils.FPSHandler()

        self.tag_dictionary = TAG_DICTIONARY
        self.valid_tags = [t['ID'] for t in self.tag_dictionary['tags']]

        self.latency = np.array([0])

        device_name = "OV2311_1"

        # camera setup
        self.camera_params = generateOV2311CameraParameters(device_name)
        self.cameraSetup()
        
        self.nt_drivetrain_tab = self.NT_Instance.getTable("Swerve")
        self.nt_apriltag_tab = self.NT_Instance.getTable(self.camera_params["nt_name"])

        self.camera_stream = CSCoreServer(self.camera, width=self.camera_params["width"], height=self.camera_params["height"], fps=self.camera_params["fps"])

        self.ip_address = 'localhost'
        self.port = 5800
        # self.mjpegServer = cscore.MjpegServer(self.ip_address, self.port)
        # self.mjpegServer.setSource(self.camera)
        # log.info("MJPEG Server started at {}:{}".format(self.ip_address, self.port))

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
        if not self.DISABLE_VIDEO_OUTPUT:
            from PyQt5 import QtWidgets
            from designer.debugWindow import DebugWindow
            self.app = QtWidgets.QApplication(sys.argv)
            self.testGui = DebugWindow(self.gyro, self.ENABLE_SOLVEPNP)
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
                if platform.system() == 'Linux':
                    retval, frame = self.camera.read()
                    timestamp = time.time_ns()
                    if not retval:
                        print("Capture session failed, restarting")
                        self.camera.release()
                        self.camera = None  # Force reconnect
                        time.sleep(2)
                        continue
                    #frame = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
                    log.info("Frame Shape: {}".format(frame.shape))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                    
                    self.process_results(frame, timestamp)
                else:
                    timestamp, frame = self.camera.getFrame()

                    self.process_results(frame, timestamp)
            else:
                self.cameraSetup()
            
    def process_results(self, frame, timestamp):
        if self.gyro is not None:
            try:
                robotAngles = {
                    'pitch': math.radians(self.gyro.get('pitch')),
                    'yaw': math.radians(self.gyro.get('yaw'))
                }
                if not self.DISABLE_VIDEO_OUTPUT:
                    self.testGui.updateYawValue(math.degrees(-robotAngles['yaw']))
                    self.testGui.updatePitchValue(math.degrees(robotAngles['pitch']))
            except Exception as e:
                # log.error("Could not grab gyro values")
                pass
        else:
            robotAngles = {
                'pitch': math.radians(self.nt_drivetrain_tab.getNumber("Yaw", 0.0)),
                'roll': math.radians(self.nt_drivetrain_tab.getNumber("Roll", 0.0)),
                'yaw': math.radians(self.nt_drivetrain_tab.getNumber("Pitch", 0.0))
            }

        monoFrame = frame
        fps = self.fps
        detector = self.detector

        fps.nextIter()
        if self.DISABLE_VIDEO_OUTPUT:
            # print("\033[2J", end='')
            pass

        if not self.DISABLE_VIDEO_OUTPUT:
            self.ENABLE_SOLVEPNP = self.testGui.getSolvePnpEnabled()

            if self.testGui.getPauseResumeState():
                monoFrame = self.lastMonoFrame

        tags = detector.detect(monoFrame)

        detectedTags = []
        x_pos = []
        y_pos = []
        z_pos = []
        pnp_tag_id = []
        pnp_x_pos = []
        pnp_y_pos = []
        pose_id = []
        if len(tags) > 0:
            for tag in tags:
                if not self.DISABLE_VIDEO_OUTPUT:
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

                wpiTransform = CoordinateSystem.convert(tagTranslation.translation(),
                                                        CoordinateSystem.EDN(),
                                                        CoordinateSystem.NWU())

                estimatedRobotPose = tagPose.transformBy(Transform3d(wpiTransform, tagTranslation.rotation()))

                tagInfo = {
                    "tag": tag,
                    "corners": tagCorners,
                    "topLeftXY": topLeftXY,
                    "bottomRightXY": bottomRightXY,
                    "tagTranslation": tagTranslation,
                    "estimatedRobotPose": estimatedRobotPose
                }

                x_pos.append(estimatedRobotPose.translation().x)
                y_pos.append(estimatedRobotPose.translation().y)
                z_pos.append(estimatedRobotPose.translation().z)
                pose_id.append(tag.getId())
                detectedTags.append(tagInfo)

        fpsValue = self.fps.fps()
        if self.lastTimestamp != 0:
            latencyMs = (timestamp - self.lastTimestamp) / 1000000.0
            self.latency = np.append(self.latency, latencyMs)
        avgLatency = np.average(self.latency) if len(self.latency) < 100 else np.average(self.latency[-100:])
        latencyStd = np.std(self.latency) if len(self.latency) < 100 else np.std(self.latency[-100:])
        self.lastTimestamp = timestamp

        self.nt_apriltag_tab.putNumberArray("tid", pose_id)
        self.nt_apriltag_tab.putNumberArray("X Poses", x_pos)
        self.nt_apriltag_tab.putNumberArray("Y Poses", y_pos)
        self.nt_apriltag_tab.putNumberArray("Z Poses", z_pos)

        # Merge AprilTag measurements to estimate
        botPose = [np.average(x_pos), np.average(y_pos), np.average(z_pos)]
        log.info("Estimated Pose X: {:.2f}\tY: {:.2f}\tZ: {:.2f}".format(botPose[0], botPose[1], botPose[2]))
        log.info("Std dev X: {:.2f}\tY: {:.2f}\tZ: {:.2f}".format(np.std(x_pos),
                                                                  np.std(y_pos),
                                                                  np.std(z_pos)))
        self.nt_apriltag_tab.putNumber("Heading Pose", 0 if robotAngles['yaw'] is None else robotAngles['yaw'])
        self.nt_apriltag_tab.putNumber("latency", self.latency[-1])

        self.nt_apriltag_tab.putNumberArray("PnP Pose ID", pnp_tag_id)
        self.nt_apriltag_tab.putNumberArray("botpose", botPose)

        self.stats = {
            'numTags': len(detectedTags),
            'depthAI': {
                'x_pos': x_pos,
                'y_pos': y_pos,
                'z_pos': z_pos,
            },
            'solvePnP': {
                'x_pos': pnp_x_pos,
                'y_pos': pnp_y_pos,
                'z_pos': [0]
            }
        }

        if not self.DISABLE_VIDEO_OUTPUT:
            for detectedTag in detectedTags:
                points = np.array([(int(p.x), int(p.y)) for p in detectedTag["corners"]])
                # Shift points since this is a snapshot
                cv2.polylines(monoFrame, [points], True, (120, 120, 120), 3)
                textX = min(points[:, 0])
                textY = min(points[:, 1]) + 20
                cv2.putText(monoFrame, "tag_id: {}".format(detectedTag['tag'].getId()),
                            (textX, textY), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))

                if self.ENABLE_SOLVEPNP:
                    r_vec = np.array([detectedTag['tagTranslation'].rotation().x,
                                      detectedTag['tagTranslation'].rotation().y,
                                      detectedTag['tagTranslation'].rotation().z])
                    t_vec = np.array([detectedTag['tagTranslation'].translation().x,
                                      detectedTag['tagTranslation'].translation().y,
                                      detectedTag['tagTranslation'].translation().z])
                    ipoints, _ = cv2.projectPoints(constants.OPOINTS,
                                                   r_vec,
                                                   t_vec,
                                                   self.camera_params['iMatrix'],
                                                   np.zeros(5))

                    ipoints = np.round(ipoints).astype(int)

                    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

                    for i, j in constants.EDGES:
                        cv2.line(monoFrame, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

                cv2.putText(monoFrame, "x: {:.2f}".format(detectedTag["tagTranslation"].translation().x),
                            (textX, textY + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                cv2.putText(monoFrame, "y: {:.2f}".format(detectedTag["tagTranslation"].translation().y),
                            (textX, textY + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                cv2.putText(monoFrame, "z: {:.2f}".format(detectedTag["tagTranslation"].translation().z),
                            (textX, textY + 60), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))

            if not self.testGui.getPauseResumeState():
                cv2.circle(monoFrame, (int(monoFrame.shape[1]/2), int(monoFrame.shape[0]/2)), 1, (255, 255, 255), 1)
                cv2.putText(monoFrame, "FPS: {:.2f}".format(fpsValue), (0, 24), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))
                cv2.putText(monoFrame, "Latency: {:.2f}ms".format(avgLatency), (0, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))

                # cv2.imshow(pipeline_info["monoRightQueue"], frameRight)
                # cv2.imshow(pipeline_info["depthQueue"], depthFrameColor)
                self.testGui.updateTagIds(pose_id)
                if len(detectedTags) > 0:
                    self.testGui.updateStatsValue(self.stats)
                self.testGui.updateFrames(monoFrame, monoFrame)
        else:
            print('FPS: {:.2f}\tLatency: {:.2f} ms\tStd: {:.2f}'.format(fpsValue, avgLatency, latencyStd))
            print('DepthAI')
            print('Tags: {}'.format(pose_id))
            print("         Avg.: {:.6f}\t{:.6f}\t{:.6f}".format(np.average(x_pos), np.average(y_pos), np.average(z_pos)))
            print("         Std.: {:.6f}\t{:.6f}\t{:.6f}".format(np.average(x_pos), np.average(y_pos), np.average(z_pos)))
            # print("\033[2J", end='')

        self.lastMonoFrame = copy.copy(monoFrame)
        self.camera_stream.setFrame(monoFrame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            raise StopIteration()
        elif key == ord(' '):
            if self.gyro is not None:
                self.gyro.resetAll()

        if not self.DISABLE_VIDEO_OUTPUT:
            if not self.testGui.isVisible():
                pass

    def init_networktables(self):
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
            log.debug("Could not connect to team client. Trying other addresses...")
            NT_Instance.setServer(secondaryIps[tries])
            tries = tries + 1

            if tries >= len(secondaryIps):
                break

        if NT_Instance.isConnected():
            log.info("NT Connected to {}".format(NT_Instance.getConnections()))
        else:
            log.error("Could not connect to NetworkTables. Restarting server...")

        return NT_Instance

    def cameraSetup(self):
        if platform.system() == 'Linux':
            self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_params["height"])
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_params["width"])
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
            self.camera.set(cv2.CAP_PROP_FPS, self.camera_params["fps"])
            self.camera.set(cv2.CAP_PROP_GAIN, self.camera_params["gain"])
            self.camera.set(cv2.CAP_PROP_EXPOSURE, -11)
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            self.camera.set(cv2.CAP_PROP_SHARPNESS, 0)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_params["height"])
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_params["width"])
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
            self.camera.set(cv2.CAP_PROP_FPS, self.camera_params["fps"])
            self.camera.set(cv2.CAP_PROP_GAIN, self.camera_params["gain"])
            self.camera.set(cv2.CAP_PROP_EXPOSURE, -11)
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            self.camera.set(cv2.CAP_PROP_SHARPNESS, 0)
            #self.camera = cv2.VideoCapture("v4l2src device=/dev/video" + str(0) + 
            #                               " extra_controls=\"c,exposure_auto=" + str(self.camera_params["exposure_auto"]) + 
            #                               ",exposure_absolute=" + str(self.camera_params["exposure"]) + 
            #                               ",gain=" + str(self.camera_params["gain"]) + 
            #                               ",sharpness=0,brightness=0\" ! image/jpeg,format=MJPG,width=" + str(self.camera_params["width"]) + 
            #                               ",height=" + str(self.camera_params["height"]) + " ! jpegdec ! video/x-raw ! appsink drop=1", cv2.CAP_GSTREAMER)
        else:
            self.camera = CSCoreCamera(self.camera_params["device_id"], True)
    

if __name__ == '__main__':
    log.info("Starting AprilTagsUSBHost")
    AprilTagsUSBHost().run()
