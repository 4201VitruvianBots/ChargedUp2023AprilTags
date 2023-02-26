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


class SensorFusionServer:

    def __init__(self):

        self.nt = self.init_networktables()

        self.nt_lLocalizer = self.nt.getTable("lLocalizer")
        self.nt_rLocalizer = self.nt.getTable("rLocalizer")
        self.nt_fusedLocalizer = self.nt.getTable("fusedLocalizer")
        self.localizers = {
            "lLocalizer": self.nt_lLocalizer,
            "rLocalizer": self.nt_rLocalizer,
        }
        self.nt_pubSub = {
            'pub': dict(),
            'sub': dict()
        }
        for name, localizer in self.localizers.items():
            sub = {
                'tv': localizer.getIntegerTopic("tv").subscribe(0),
                'tid': localizer.getIntegerArrayTopic("tid").subscribe([0]),
                'rPosX': localizer.getDoubleArrayTopic("Robot Pose X").subscribe([0]),
                'rPosY': localizer.getDoubleArrayTopic("Robot Pose Y").subscribe([0]),
                'rPosYaw': localizer.getDoubleArrayTopic("Robot Pose Yaw").subscribe([0]),
                'latency': localizer.getDoubleArrayTopic("latency").subscribe([0]),
            }
            self.nt_pubSub['sub'][name] = sub
        self.nt_pubSub['pub']['fusedLocalizer'] = {
            'tv': localizer.getIntegerTopic("tv").publish(),
            'tid': localizer.getIntegerArrayTopic("tid").publish(),
            'rPosX': self.nt_fusedLocalizer.getDoubleArrayTopic("Robot Pose X").publish(),
            'rPosY': self.nt_fusedLocalizer.getDoubleArrayTopic("Robot Pose Y").publish(),
            'rPosYaw': self.nt_fusedLocalizer.getDoubleArrayTopic("Robot Pose Yaw").publish(),
            'latency': self.nt_fusedLocalizer.getDoubleArrayTopic("latency").publish(),
            'botpose': self.nt_fusedLocalizer.getDoubleArrayTopic("botpose").publish(),
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
            t_ids = []
            x_poses = []
            y_poses = []
            yaw_rotations = []
            latencies = []
            for name, localizer in self.nt_pubSub['sub'].items():
                tv = localizer['tv'].get()
                local_ids = localizer['tid'].get()
                if not tv:
                    continue
                if len(local_ids) == 0:
                    continue
                if local_ids[0] == -1:
                    continue

                local_l = localizer['latency'].get([0])
                local_x = localizer['rPosX'].get([0])
                local_y = localizer['rPosY'].get([0])
                local_yaw = localizer['rPosYaw'].get([0])

                if not (len(local_ids) == len(local_x) == len(local_y) == len(local_yaw) == len(local_l)):
                    log.warning("NTServerError::{} array values are not the same length. Skipping...")
                    continue

                t_ids.extend(local_ids)
                x_poses.extend(local_x)
                y_poses.extend(local_y)
                yaw_rotations.extend(local_yaw)
                latencies.extend(local_l)

            if len(t_ids) == 0:
                continue

            x_mean = np.mean(x_poses)
            y_mean = np.mean(y_poses)
            yaw_mean = np.mean(yaw_rotations)
            x_std = np.std(x_poses)
            y_std = np.std(y_poses)
            yaw_std = np.std(yaw_rotations)
            std_multiplier = 1.1
            rm_idx = []
            if len(y_poses) > 1:
                for pose_idx in range(len(t_ids)):
                    if math.fabs(x_poses[pose_idx] - x_mean) > x_std * std_multiplier:
                        rm_idx.append(pose_idx)
                    if math.fabs(x_poses[pose_idx] - y_mean) > y_std * std_multiplier:
                        rm_idx.append(pose_idx)
                    if math.fabs(x_poses[pose_idx] - yaw_mean) > yaw_std * std_multiplier:
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

                self.nt_pubSub['pub']['fusedLocalizer']['tv'].set(1 if len(t_ids) > 0 else 0)
                self.nt_pubSub['pub']['fusedLocalizer']['tid'].set(t_ids)
                self.nt_pubSub['pub']['fusedLocalizer']['rPosX'].set(robot_pose_x)
                self.nt_pubSub['pub']['fusedLocalizer']['rPosY'].set(robot_pose_y)
                self.nt_pubSub['pub']['fusedLocalizer']['rPosYaw'].set(robot_pose_yaw)
                self.nt_pubSub['pub']['fusedLocalizer']['latency'].set(robot_pose_latency)
                self.nt_pubSub['pub']['fusedLocalizer']['botpose'].set(botPose)

                log.debug("Fused Pose estimation: {}".format(botPose))


if __name__ == '__main__':
    SensorFusionServer()
