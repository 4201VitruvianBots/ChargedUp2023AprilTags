import argparse
import copy
from time import sleep

import cv2
import depthai as dai
import logging
import math
import numpy as np
from os.path import basename
import socket
import sys

from ntcore import NetworkTableInstance

from common import constants
from common import utils
from common.cscoreServer import CSCoreServer
from common.depthaiUtils import generateCameraParameters
from common.imu import navX
from aprilTags.tag_dictionary import TAG_DICTIONARY
from pipelines import apriltags_pipeline

from pipelines.apriltags_pipeline import create_dual_mono_apriltag_pipeline

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


class AprilTagsHost:
    initialized = False

    def __init__(self):
        self.NT_Instance = self.init_networktables()
        if args.performance_test:
            disabledStdOut = logging.StreamHandler(stream=None)
            log.addHandler(disabledStdOut)

        log.info("Starting AprilTag Spatial Detector")

        self.DISABLE_VIDEO_OUTPUT = args.test
        self.ENABLE_SOLVEPNP = args.apriltag_pose

        self.pipeline, self.pipeline_info = create_dual_mono_apriltag_pipeline()

        self.valid_cameras = None
        if args.position is not None:
            self.valid_cameras = constants.CAMERAS[args.p]

        self.nt_drivetrain_tab = self.NT_Instance.getTable("Swerve")

        self.fps = utils.FPSHandler()

        self.tag_dictionary = TAG_DICTIONARY
        self.valid_tags = [t['ID'] for t in self.tag_dictionary['tags']]

        self.gyro = None
        if self.USE_EXTERNAL_IMU:
            try:
                self.gyro = navX('COM4')
            except Exception as e:
                log.error("Could not initialize gyro")

        self.latency = np.array([])

        found = False
        device_info = None
        while not found:
            if self.valid_cameras is None:
                found, device_info = dai.Device.getFirstAvailableDevice()
            else:
                for device_id in self.valid_cameras:
                    found, device_info = dai.Device.getDeviceByMxId(device_id)
                    if found:
                        break

            if found:
                log.info("Camera {} found".format(device_info.getMxId()))

                self.camera_params = generateCameraParameters(self.pipeline, self.pipeline_info, device_info)
                self.nt_depthai_tab = self.NT_Instance.getTable(self.camera_params['nt_name'])
                break
            else:
                log.error("No Cameras found. Polling again...")
                sleep(1)

        self.camera_stream = CSCoreServer(self.camera_params['device_name'])
        # self.camera_stream = CSCoreServer(self.camera_params['device_name'])
        self.lastMonoFrame = None
        self.lastDepthFrame = None
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
            while True:
                if not self.DISABLE_VIDEO_OUTPUT:
                    found, device_info = dai.Device.getDeviceByMxId(self.camera_params['id'])
                    if found:
                        try:
                            self.detect_apriltags(device_info)
                        except Exception as e:
                            pass
                    else:
                        log.warning("No DepthAI Camera found")
                else:
                    if self.run_thread is None or not self.run_thread.is_alive():
                        found, device_info = dai.Device.getDeviceByMxId(self.camera_params['id'])
                        self.nt_depthai_tab.putBoolean("Localizer Status", found)

                        if found:
                            log.info("Camera {} found. Starting processing thread...".format(self.camera_params['id']))

                            while True:
                                try:
                                    self.detect_apriltags(device_info)
                                except Exception as e:
                                    break

                            # self.run_thread = threading.Thread(target=self.detect_apriltags, args=(device_info,))
                            # self.run_thread.daemon = True
                            # self.run_thread.start()
                        else:
                            log.error("Camera {} not found. Attempting to restart thread...".format(self.camera_params['id']))

                    if self.run_thread is not None and not self.run_thread.is_alive():
                        log.error("{} thread died. Restarting thread...".format(self.camera_params['device_type']))
                sleep(0.5)
        except KeyboardInterrupt as e:
            log.info("Keyboard Interrupt")
            if self.run_thread is not None:
                self.run_thread.join()

    def detect_apriltags(self, device):
        for returnCode, leftFrame, rightFrame, timestampLns, timestampRns in apriltags_pipeline.dual_capture(device, self.camera_settings):
            if returnCode == -1:
                raise IOError("DepthAI Fatal Crash")
            self.process_results(leftFrame, rightFrame, timestampLns, timestampRns)

    def process_results(self, leftFrame, rightFrame, timestampLns, timestampRns):
        frames = [leftFrame, rightFrame]
        timestamps = [timestampLns, timestampRns]

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

        for i in range(len(self.camera_params['cameras'])):
            monoFrame = frames[i]
            fps = self.camera_params['cameras'][i]['fps']
            timestamp = timestamps[i]
            detector = self.camera_params['cameras'][i]['detector']

            fps.nextIter()
            if self.DISABLE_VIDEO_OUTPUT:
                # print("\033[2J", end='')
                pass

            if not self.DISABLE_VIDEO_OUTPUT:
                self.ENABLE_SOLVEPNP = self.testGui.getSolvePnpEnabled()

                if self.testGui.getPauseResumeState():
                    depthFrame = self.lastDepthFrame
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

                    roi = (topLeftXY[0], topLeftXY[1], bottomRightXY[0], bottomRightXY[1])

                    tagDictionaryPose = self.tag_dictionary['tags'][tag.getId() - 1]['pose']['translation']
                    pnpRobotPose = detector.estimatePose(tag)


            fpsValue = self.fps.fps()
            latencyMs = (dai.Clock.now() - timestamp).total_seconds() * 1000.0
            self.latency = np.append(self.latency, latencyMs)
            avgLatency = np.average(self.latency) if len(self.latency) < 100 else np.average(self.latency[-100:])
            latencyStd = np.std(self.latency) if len(self.latency) < 100 else np.std(self.latency[-100:])

            self.nt_depthai_tab.putNumberArray("tid", pose_id)
            self.nt_depthai_tab.putNumberArray("X Poses", x_pos)
            self.nt_depthai_tab.putNumberArray("Y Poses", y_pos)
            self.nt_depthai_tab.putNumberArray("Z Poses", z_pos)

            # Merge AprilTag measurements to estimate
            botPose = [np.average(x_pos), np.average(y_pos), np.average(z_pos)]
            log.info("Estimated Pose X: {:.2f}\tY: {:.2f}\tZ: {:.2f}".format(botPose[0], botPose[1], botPose[2]))
            log.info("Std dev X: {:.2f}\tY: {:.2f}\tZ: {:.2f}".format(np.std(x_pos),
                                                                      np.std(y_pos),
                                                                      np.std(z_pos)))
            self.nt_depthai_tab.putNumber("Avg X Pose", np.average(x_pos))
            self.nt_depthai_tab.putNumber("Avg Y Pose", np.average(y_pos))
            self.nt_depthai_tab.putNumber("Heading Pose", 0 if robotAngles['yaw'] is None else robotAngles['yaw'])
            self.nt_depthai_tab.putNumber("latency", self.latency[-1])

            self.nt_depthai_tab.putNumberArray("PnP Pose ID", pnp_tag_id)
            self.nt_depthai_tab.putNumberArray("PnP X Poses", pnp_x_pos)
            self.nt_depthai_tab.putNumberArray("PnP Y Poses", pnp_y_pos)
            self.nt_depthai_tab.putNumberArray("botpose", botPose)

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
                        r_vec = np.array([detectedTag['tagPose'].rotation().x,
                                          detectedTag['tagPose'].rotation().y,
                                          detectedTag['tagPose'].rotation().z])
                        t_vec = np.array([detectedTag['tagPose'].translation().x,
                                          detectedTag['tagPose'].translation().y,
                                          detectedTag['tagPose'].translation().z])
                        ipoints, _ = cv2.projectPoints(constants.OPOINTS,
                                                       r_vec,
                                                       t_vec,
                                                       self.camera_params['iMatrix'],
                                                       np.zeros(5))

                        ipoints = np.round(ipoints).astype(int)

                        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

                        for i, j in constants.EDGES:
                            cv2.line(monoFrame, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

                    cv2.putText(monoFrame, "x: {:.2f}".format(detectedTag["spatialData"]['x']),
                                (textX, textY + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                    cv2.putText(monoFrame, "y: {:.2f}".format(detectedTag["spatialData"]['y']),
                                (textX, textY + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                    cv2.putText(monoFrame, "x angle: {:.2f}".format(detectedTag["translation"]['x_angle']),
                                (textX, textY + 60), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                    cv2.putText(monoFrame, "y angle: {:.2f}".format(detectedTag["translation"]['y_angle']),
                                (textX, textY + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                    cv2.putText(monoFrame, "z: {:.2f}".format(detectedTag["spatialData"]['z']),
                                (textX, textY + 100), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
                    cv2.rectangle(monoFrame, detectedTag["topLeftXY"], detectedTag["bottomRightXY"],
                                  (0, 0, 0), 3)

                if not self.testGui.getPauseResumeState():
                    cv2.circle(monoFrame, (int(monoFrame.shape[1]/2), int(monoFrame.shape[0]/2)), 1, (255, 255, 255), 1)
                    cv2.putText(monoFrame, "FPS: {:.2f}".format(fpsValue), (0, 24), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))
                    cv2.putText(monoFrame, "Latency: {:.2f}ms".format(avgLatency), (0, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))

                    depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    depthFrameColor = cv2.equalizeHist(depthFrameColor)
                    depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                    # cv2.imshow(pipeline_info["monoRightQueue"], frameRight)
                    # cv2.imshow(pipeline_info["depthQueue"], depthFrameColor)
                    self.testGui.updateTagIds(pose_id)
                    if len(detectedTags) > 0:
                        self.testGui.updateStatsValue(self.stats)
                    self.testGui.updateFrames(monoFrame, depthFrameColor)
            else:
                print('FPS: {:.2f}\tLatency: {:.2f} ms\tStd: {:.2f}'.format(fpsValue, avgLatency, latencyStd))
                print('DepthAI')
                print('Tags: {}'.format(pose_id))
                print("         Avg.: {:.6f}\t{:.6f}\t{:.6f}".format(np.average(x_pos), np.average(y_pos), np.average(z_pos)))
                print("         Std.: {:.6f}\t{:.6f}\t{:.6f}".format(np.average(x_pos), np.average(y_pos), np.average(z_pos)))
                # print("\033[2J", end='')

            self.lastMonoFrame = copy.copy(monoFrame)
            self.lastDepthFrame = copy.copy(depthFrame)
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


if __name__ == '__main__':
    log.info("Starting AprilTagsHost")
    AprilTagsHost().run()
