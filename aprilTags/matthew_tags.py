import copy
import math
import pathlib
import socket
import sys
from os.path import basename

import cv2
import os
import time
import typing

import numpy as np
import robotpy_apriltag

from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Quaternion, CoordinateSystem, Transform3d

import aprilTags.tag_dictionary
from aprilTags.UsbHost import log, args
from common import constants, utils
from common.cvUtils import TargetDrawer


def init_network_tables():
    nt_instance = NetworkTableInstance.getDefault()
    identity = f"{basename(__file__)}-{os.getpid()}"
    nt_instance.startClient4(identity)
    nt_instance.setServer("10.42.1.2")
    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)

    secondary_ips = [
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
    while not nt_instance.isConnected():
        log.debug("Could not connect to team client. Trying other addresses...")
        nt_instance.setServer(secondary_ips[tries])

        time.sleep(1)
        if nt_instance.isConnected():
            log.info("Found NT Server at {}".format(secondary_ips[tries]))
            break
        tries += 1
        if tries >= len(secondary_ips):
            log.error("Could not connect to NetworkTables...")
            break

    return nt_instance


nt_instance = init_network_tables()
nt_drivetrain_tab = nt_instance.getTable("Swerve")
nt_apriltag_tab = nt_instance.getTable("Shuffleboard")
nt_subs = {
    'Pitch': nt_drivetrain_tab.getDoubleTopic("Pitch").subscribe(0),
    'Roll': nt_drivetrain_tab.getDoubleTopic("Roll").subscribe(0),
    'Yaw': nt_drivetrain_tab.getDoubleTopic("Yaw").subscribe(0),
    'camToRobotT3D': nt_apriltag_tab.getDoubleArrayTopic("camToRobotT3D").subscribe([0, 0, 0, 0, 0, 0]),
}

nt_pubs = {
    'tv': nt_apriltag_tab.getIntegerTopic("tv").publish(),
    'tid': nt_apriltag_tab.getIntegerArrayTopic("tid").publish(),
    'rPosX': nt_apriltag_tab.getDoubleArrayTopic("Robot Pose X").publish(),
    'rPosY': nt_apriltag_tab.getDoubleArrayTopic("Robot Pose Y").publish(),
    'rPosYaw': nt_apriltag_tab.getDoubleArrayTopic("Robot Pose Yaw").publish(),
    'tPosX': nt_apriltag_tab.getDoubleArrayTopic("Tag Pose X").publish(),
    'tPosY': nt_apriltag_tab.getDoubleArrayTopic("Tag Pose Y").publish(),
    'latency': nt_apriltag_tab.getDoubleTopic("latency").publish(),
    'bot-pose': nt_apriltag_tab.getDoubleArrayTopic("bot-pose").publish(),
}


def save_frame_every_second(output_folder: pathlib.Path, capture_duration: float = 60):
    # Ensure the output folder exists
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the camera (0 is the default camera index)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Camera not found.")
        return

    start_time = time.time()
    frame_counter = 0

    while int(time.time() - start_time) < capture_duration:
        # Read a frame from the camera
        image_file_name = input('image filename:')

        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        current_time = int(time.time())
        if current_time % 1 == 0 and frame_counter != current_time:
            # Save the frame to the output folder
            output_path = output_folder / f"{image_file_name}.png"
            cv2.imwrite(str(output_path), frame)
            print(f"Saved frame {frame_counter} to {output_path}")
            frame_counter += 1

        # Sleep for a short while to avoid high CPU usage
        time.sleep(0.1)

    # Release the camera resources
    cap.release()


def loop_save_frames_with_text():
    output_folder = pathlib.Path("aprilTags/captured_frames")
    capture_duration = 60  # Capture images for 60 seconds
    save_frame_every_second(output_folder, capture_duration)


def show_frame():
    frame_directory = pathlib.Path('aprilTags/captured_frames')
    for image_file in frame_directory.glob('*.png'):
        image = cv2.imread(str(image_file))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        detector = robotpy_apriltag.AprilTagDetector()

        tag_size_meters = 0.1524

        detector_intrinsics = robotpy_apriltag.AprilTagPoseEstimator.Config(
            tag_size_meters,
            8.7638714483680690e+02,
            8.7644598649542741e+02,
            8.4331555653705630e+02,
            6.5641811849711667e+02)

        estimator = robotpy_apriltag.AprilTagPoseEstimator(detector_intrinsics)

        # detector config
        detector_config = robotpy_apriltag.AprilTagDetector.Config()
        detector_config.refineEdges = 1.0
        detector_config.quadDecimate = 1.0
        detector_config.numThreads = 3
        detector_config.quadSigma = 1.0
        detector_config.decodeSharpening = 0.25
        detector.setConfig(detector_config)
        detector.addFamily("tag16h5", 3)
        detections = detector.detect(image)

        # calls tag filter
        valid_detections = tag_filter(detections)

        # calls tag estimator
        estimated_detections = tag_estimate(valid_detections, estimator)
        print(estimated_detections)

        # cycle images
        cv2.imshow(str(image_file), image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def tag_filter(detections: typing.List[robotpy_apriltag.AprilTagDetection.Point]):
    # cycles through every detected tag and prints detection corners
    tag_threshold = 7

    valid_detections = []
    if len(detections) > 0:
        for tag in detections:
            if tag.getId() not in range(1, 8):
                continue

            tag_corners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]

            # finds diff of right/left sides
            right_side = point_dist(tag_corners[3], tag_corners[0])
            left_side = point_dist(tag_corners[2], tag_corners[1])
            side_diff = abs(right_side - left_side)
            if side_diff > tag_threshold:
                continue

            # finds diff of top/bottom sides
            top = point_dist(tag_corners[3], tag_corners[2])
            bottom = point_dist(tag_corners[1], tag_corners[0])
            side_diff = abs(top - bottom)
            if side_diff > tag_threshold:
                continue

            # checks if valid tag is within reasonable aspect ratio
            aspect = top / right_side
            if not (0.8 < aspect < 1.2):
                continue

            # checks if valid tag is within reasonable area
            area = top * right_side
            if area < 96:
                continue

            # ic(tag_corners)
            # ic(side_diff)
            # print(tag)
            valid_detections.append(tag)

    return valid_detections


def tag_estimate(valid_detections, estimator):
    # network table setup
    device_name = args.dev
    camera_params = generate_camera_parameters(device_name)

    camera_to_robot_v = nt_subs['camToRobotT3D'].get()
    camera_to_robot_transform = Transform3d(
        Translation3d(camera_to_robot_v[0], camera_to_robot_v[1], camera_to_robot_v[2]),
        Rotation3d(camera_to_robot_v[3], camera_to_robot_v[4], camera_to_robot_v[5]))

    # detected tag setup
    detected_tags = []
    robot_pose_x = []
    robot_pose_y = []
    robot_pose_z = []
    robot_pose_yaw = []

    tag_id = []
    tag_pose_x = []
    tag_pose_y = []
    tag_pose_z = []

    # estimate tag to robot
    if len(valid_detections) > 0:
        for detection in valid_detections:
            # wpi math
            tag_values = aprilTags.tag_dictionary.TAG_DICTIONARY['tags'][detection.getId() - 1]['pose']
            tag_translation = tag_values['translation']
            tag_rotation = tag_values['rotation']['quaternion']
            tag_translation_3d = Translation3d(tag_translation['x'], tag_translation['y'], tag_translation['z'])
            tag_rotation_3d = Rotation3d(
                Quaternion(tag_rotation['W'], tag_rotation['X'], tag_rotation['Y'], tag_rotation['Z']))
            tag_pose = Pose3d(tag_translation_3d, tag_rotation_3d)
            camera_to_tag_estimate = estimator.estimate(detection)

            # camera rotation
            cam_rotation = camera_to_tag_estimate.rotation()
            cam_inv_rotation = camera_to_tag_estimate.inverse().rotation()
            rotated_camera_to_tag_estimate = Transform3d(camera_to_tag_estimate.translation(), cam_rotation)

            # changes tag coordinate system to wpi system
            wpi_translation = CoordinateSystem.convert(camera_to_tag_estimate.translation().rotateBy(cam_inv_rotation),
                                                       CoordinateSystem.EDN(),
                                                       CoordinateSystem.NWU())

            # robot pose
            estimated_robot_transform = tag_pose.transformBy(Transform3d(wpi_translation, Rotation3d())).transformBy(
                camera_to_robot_transform)
            estimated_robot_pose = Pose3d(estimated_robot_transform.translation(), cam_rotation)

            tag_corners = [detection.getCorner(0), detection.getCorner(1), detection.getCorner(2),
                           detection.getCorner(3)]

            # detected tag data
            detected_tag = {
                "tag": detection,
                "corners": tag_corners,
                "tagPose": tag_pose,
                "tagTranslation": rotated_camera_to_tag_estimate,
                "wpiCameraToTag": estimated_robot_transform,
                "estimatedRobotPose": estimated_robot_pose,
                'robot_pose_x': robot_pose_x,
                'robot_pose_y': robot_pose_y,
                'robot_pose_z': robot_pose_z,
            }

            detected_tags.append(detected_tag)

    # adding estimated pose from tag
    detected_tags = sorted(detected_tags, key=lambda d: d['tag'].getDecisionMargin(), reverse=True)
    for detectedTag in detected_tags:
        robot_pose_x.append(detectedTag["estimatedRobotPose"].translation().x)
        robot_pose_y.append(detectedTag["estimatedRobotPose"].translation().y)
        robot_pose_yaw.append(detectedTag["estimatedRobotPose"].rotation().y_degrees)
        tag_pose_x.append(detectedTag["tagPose"].translation().x)
        tag_pose_y.append(detectedTag["tagPose"].translation().y)
        tag_pose_z.append(detectedTag["tagPose"].translation().z)
        tag_id.append(detectedTag["tag"].getId())

    # latency for wpi
    latency = np.array([0])
    last_time_stamp = 0
    time_stamp = time.time_ns()
    fps = utils.FPSHandler()
    fps_value = fps.fps()
    if last_time_stamp != 0:
        latency_ms = (time_stamp - last_time_stamp) / 1000000.0
        latency = np.append(latency, latency_ms)
    avg_latency = np.average(latency) if len(latency) < 100 else np.average(latency[-100:])
    last_time_stamp = time_stamp

    # average bot pose
    bot_pose = [np.average(robot_pose_x), np.average(robot_pose_y), 0,
                0, 0, np.average(robot_pose_yaw),
                latency[-1]]

    # publish pose data on network table
    nt_pubs["tv"].set(1 if len(tag_id) > 0 else 0)
    nt_pubs["tid"].set(tag_id)
    nt_pubs["rPosX"].set(robot_pose_x)
    nt_pubs["rPosY"].set(robot_pose_y)
    nt_pubs["rPosYaw"].set(robot_pose_yaw)
    nt_pubs["tPosX"].set(tag_pose_x)
    nt_pubs["tPosY"].set(tag_pose_y)
    nt_pubs["latency"].set(latency[-1])
    nt_pubs["bot-pose"].set(bot_pose)

    return detected_tags


def point_dist(p1: robotpy_apriltag.AprilTagDetection.Point, p2: robotpy_apriltag.AprilTagDetection.Point):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

def generate_camera_parameters(device_id):
    device_type = 'Left_Localizers'
    if device_id in constants.CAMERAS['Right_Localizers']['ids'].keys():
        device_type = 'Right_Localizers'
    device_params = constants.CAMERAS[device_type]
    device_name = device_params['ids'][device_id]

    i_matrix = constants.CAMERA_PARAMS[device_name]["camera_matrix"][device_id]

    if i_matrix.shape == (1, 9) or i_matrix.shape == (9, 1):
        i_matrix.reshape([3, 3])

    if i_matrix.shape != (3, 3):
        print("ERROR: camera iMatrix is not a 3x3 matrix!")

    hfov = constants.CAMERA_PARAMS[device_name]["mono"]["hfov"]
    vfov = constants.CAMERA_PARAMS[device_name]["mono"]["vfov"]

    camera_params = {
        "device_id": device_id,
        "device_name": device_name,
        "device_type": device_type,
        "id": 0,
        "nt_name": device_params['nt_name'],
        "hfov": hfov,
        "vfov": vfov,
        "height": constants.CAMERA_PARAMS[device_name]["height"],
        "width": constants.CAMERA_PARAMS[device_name]["width"],
        "fps": constants.CAMERA_PARAMS[device_name]["fps"],
        "pixelFormat": constants.CAMERA_PARAMS[device_name]["pixelFormat"],
        "exposure_auto": 0,
        "exposure": 0.1,
        "gain": 200,
        "mount_angle_radians": math.radians(device_params['mount_angle'][0]),
        "iMatrix": np.array(i_matrix).reshape(3, 3),
        # fx, fy, cx, cy
        "intrinsicValues": (i_matrix[0][0], i_matrix[1][1], i_matrix[0][2], i_matrix[1][2]),
        "hfl": constants.CAMERA_PARAMS[device_name]["width"] / (2 * math.tan(math.radians(hfov) / 2)),
        "vfl": constants.CAMERA_PARAMS[device_name]["height"] / (2 * math.tan(math.radians(vfov) / 2))
    }

    return camera_params


if __name__ == "__main__":
    show_frame()
