import argparse

import cv2
import depthai as dai
import numpy as np
import robotpy_apriltag

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
    pipeline = apriltags_pipeline.create_hires_apriltag_pipeline()
    # pipeline = apriltags_pipeline.create_mono_apriltag_pipeline()

    detector = robotpy_apriltag.AprilTagDetector()
    detectorConfig = robotpy_apriltag.AprilTagDetector.Config()
    detectorConfig.refineEdges = args.refine_edges
    detectorConfig.quadDecimate = args.quad_decimate
    detectorConfig.numThreads = args.nthreads
    detectorConfig.quadSigma = args.quad_sigma
    detectorConfig.decodeSharpening = args.decode_sharpening
    detector.setConfig(detectorConfig)
    detector.addFamily(args.family, 0)

    fps = utils.FPSHandler()
    latency = np.array([])

    with dai.Device(pipeline) as device:
        outputQueue = device.getOutputQueue(name="monoRgb", maxSize=1, blocking=False)

        deviceID = device.getMxId()
        eepromData = device.readCalibration().getEepromData()
        iMatrix = eepromData.cameraData.get(dai.CameraBoardSocket.RIGHT).intrinsicMatrix

        if deviceID in constants.CAMERA_IDS:
            productName = constants.CAMERA_IDS[deviceID]['name']
        else:
            productName = eepromData.productName

            if len(productName) == 0:
                boardName = eepromData.boardName
                if boardName == 'BW1098OBC':
                    productName = 'OAK-D'
                else:
                    productName = 'OAK-D'

        camera_params = {
            "hfov": constants.CAMERA_PARAMS[productName]["mono"]["hfov"],
            "vfov": constants.CAMERA_PARAMS[productName]["mono"]["vfov"],
            "iMatrix": np.array(iMatrix).reshape(3, 3),
            # fx, fy, cx, cy
            "intrinsicValues": (iMatrix[0][0], iMatrix[1][1], iMatrix[0][2], iMatrix[1][2])
        }
        detectorIntrinsics = robotpy_apriltag.AprilTagPoseEstimator.Config(
            constants.TAG_SIZE_M,
            camera_params['intrinsicValues'][0],
            camera_params['intrinsicValues'][1],
            camera_params['intrinsicValues'][2],
            camera_params['intrinsicValues'][3])

        detectorPoseEstimator = robotpy_apriltag.AprilTagPoseEstimator(detectorIntrinsics)

        while True:
            try:
                outputFrame = outputQueue.get()  # blocking call, will wait until a new data has arrived
                fps.nextIter()
            except Exception as e:
                continue
            inMono = outputFrame.getCvFrame()
            tags = detector.detect(inMono)

            fpsValue = fps.fps()
            latencyMs = (dai.Clock.now() - outputFrame.getTimestamp()).total_seconds() * 1000.0
            latency = np.append(latency, latencyMs)
            avgLatency = np.average(latency) if len(latency) < 100 else np.average(latency[-100:])
            latencyStd = np.std(latency) if len(latency) < 100 else np.std(latency[-100:])

            if len(tags) > 0:
                for tag in tags:
                    if tag.getId() not in range(1, 9):
                        continue

                    tagCorners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]
                    tagPose = detectorPoseEstimator.estimate(tag)

                    r_vec = np.array([tagPose.rotation().x,
                                      tagPose.rotation().y,
                                      tagPose.rotation().z])
                    t_vec = np.array([tagPose.translation().x,
                                      tagPose.translation().y,
                                      tagPose.translation().z])
                    ipoints, _ = cv2.projectPoints(constants.OPOINTS,
                                                   r_vec,
                                                   t_vec,
                                                   camera_params['iMatrix'],
                                                   np.zeros(5))

                    ipoints = np.round(ipoints).astype(int)

                    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

                    for i, j in constants.EDGES:
                        cv2.line(inMono, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

                    points = np.array([(int(p.x), int(p.y)) for p in tagCorners])
                    # Shift points since this is a snapshot
                    cv2.polylines(inMono, [points], True, (120, 120, 120), 1)
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
            cv2.imshow("HiRes", inMono)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break


if __name__ == '__main__':
    main()
