import robotpy_apriltag

from common import constants


class AprilTagDetector:
    def __init__(self, args, camera_params=None):
        self.detector = robotpy_apriltag.AprilTagDetector()
        self.detectorConfig = robotpy_apriltag.AprilTagDetector.Config()
        self.detectorConfig.refineEdges = args.refine_edges
        self.detectorConfig.quadDecimate = args.quad_decimate
        self.detectorConfig.numThreads = args.nthreads
        self.detectorConfig.quadSigma = args.quad_sigma
        self.detectorConfig.decodeSharpening = args.decode_sharpening
        self.detector.setConfig(self.detectorConfig)
        self.detector.addFamily(args.family, 0)

        self.poseEstimator = None
        if camera_params is not None:
            self.poseEstimator = self.setCamperaParams(camera_params)

    def setCamperaParams(self, camera_params):
        detectorIntrinsics = robotpy_apriltag.AprilTagPoseEstimator.Config(
            constants.TAG_SIZE_M,
            camera_params['intrinsicValues'][0],
            camera_params['intrinsicValues'][1],
            camera_params['intrinsicValues'][2],
            camera_params['intrinsicValues'][3])

        return robotpy_apriltag.AprilTagPoseEstimator(detectorIntrinsics)

    def detect(self, frame):
        return self.detector.detect(frame)

    def estimatePose(self, tag):
        if self.poseEstimator is not None:
            return self.poseEstimator.estimate(tag)
        else:
            return None
