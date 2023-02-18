import logging
import math
import threading
from os.path import basename
import socket
from time import sleep

import numpy as np
from ntcore import NetworkTableInstance

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.INFO)

class NTServer:

    def __init__(self):

        self.nt = self.init_networktables()

        self.nt_lLocalizer = self.nt.getTable("lLocalizer")
        self.nt_rLocalizer = self.nt.getTable("rLocalizer")
        self.nt_fusedLocalizer = self.nt.getTable("fusedLocalizer")
        self.localizers = {
            "lLocalizer": self.nt_lLocalizer,
            "rLocalizer": self.nt_rLocalizer,
        }
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        t.join()

    def init_networktables(self):
        NT_Instance = NetworkTableInstance.getDefault()
        identity = basename(__file__)
        NT_Instance.startClient4(identity)
        NT_Instance.setServer("10.42.1.2")
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)

        secondaryIps = [
                ip,
                'localhost',
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
        sleep(1)
        while not NT_Instance.isConnected():
            log.debug("Could not connect to team client. Trying other addresses...")
            NT_Instance.setServer(secondaryIps[tries])
            tries += 1

            sleep(1)
            if NT_Instance.isConnected():
                break

            if tries >= len(secondaryIps):
                break

        if NT_Instance.isConnected():
            log.info("NT Connected to {}".format(NT_Instance.getConnections()))
        else:
            log.error("Could not connect to NetworkTables. Restarting server...")

        return NT_Instance

    def _run(self):
        while True:
            latencies = []
            t_ids = []
            x_poses = []
            y_poses = []
            yaw_rotations = []
            for name, localizer in self.localizers.items():
                local_ids = localizer.getNumberArray("tid", [-1])
                if len(local_ids) == 0:
                    continue
                if local_ids[0] == -1:
                    continue

                local_x = localizer.getNumberArray("Robot Pose X", [0])
                local_y = localizer.getNumberArray("Robot Pose Y", [0])
                local_yaw = localizer.getNumberArray("Robot Pose Yaw", [0])

                if not (len(local_ids) == len(local_x) == len(local_y) == len(local_yaw)):
                    log.warning("NTServerError::{} array values are not the same length. Skipping...")
                    continue

                local_latency = localizer.getNumber("latency", 0)
                lArray = np.ones([1, len(local_ids)], dtype=float) * local_latency
                try:
                    latencies.extend(lArray[0].tolist())
                except Exception as e:
                    continue
                t_ids.extend(local_ids)
                x_poses.extend(local_x)
                y_poses.extend(local_y)
                yaw_rotations.extend(local_yaw)

            if len(t_ids) == 0:
                continue

            x_mean = np.mean(x_poses)
            y_mean = np.mean(y_poses)
            yaw_mean = np.mean(yaw_rotations)
            x_std = np.std(x_poses)
            y_std = np.std(y_poses)
            yaw_std = np.std(yaw_rotations)
            std_multiplier = 0.25
            rm_idx = []
            if len(y_poses) > 1:
                for pose_idx in range(len(t_ids)):
                    if math.fabs(x_poses[pose_idx] - x_mean) > x_std * std_multiplier:
                        rm_idx.append(pose_idx)
                    if math.fabs(x_poses[pose_idx] - y_mean) > y_std * std_multiplier:
                        rm_idx.append(pose_idx)
                    if math.fabs(x_poses[pose_idx] - yaw_mean) > yaw_std:
                        rm_idx.append(pose_idx)

                rm_idx = np.unique(rm_idx)
                t_ids = [i for j, i in enumerate(t_ids) if j not in rm_idx]
                robot_pose_x = [i for j, i in enumerate(x_poses) if j not in rm_idx]
                robot_pose_y = [i for j, i in enumerate(y_poses) if j not in rm_idx]
                robot_pose_yaw = [i for j, i in enumerate(yaw_rotations) if j not in rm_idx]
                robot_pose_latency = [i for j, i in enumerate(latencies) if j not in rm_idx]
                avg_lat = np.average(robot_pose_latency)
                botPose = [np.average(robot_pose_x), np.average(robot_pose_y), 0, 0, 0,
                           np.average(robot_pose_yaw), avg_lat]

                self.nt_fusedLocalizer.putNumberArray("botpose", botPose)
                self.nt_fusedLocalizer.putNumber("latency", avg_lat)
                log.debug("Fused Pose estimation: {}".format(botPose))


if __name__ == '__main__':
    NTServer()
