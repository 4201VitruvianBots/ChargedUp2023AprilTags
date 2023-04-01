
from wpimath.geometry import Transform3d, CoordinateSystem, Translation3d, Quaternion, Rotation3d, Pose3d
import cv2
import copy
import glob
import math
import numpy as np
import robotpy_apriltag

from aprilTags.apriltagDetector import AprilTagDetector
from aprilTags.tag_dictionary import TAG_DICTIONARY
from common import cvUtils
from common.cvUtils import TargetDrawer
from cscore_utils.usbCameraUtils import generateCameraParameters

Point = robotpy_apriltag.AprilTagDetection.Point

folderDirectory = 'resources/CW_Rotation.png'

searchRegex = r"{}".format(folderDirectory)
imageFile = glob.glob(searchRegex)[0]
image = cv2.imread(imageFile)

camera_params = generateCameraParameters("OV2311")


class AprilTagParameters:
    refine_edges = 1.0
    quad_decimate = 2
    quad_sigma = 0
    nthreads = 3
    decode_sharpening = 0.25
    family = 'tag16h5'


args = AprilTagParameters

detector = AprilTagDetector(args, camera_params)
cameraToRobotTransform = Transform3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))

image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
image2 = copy.copy(image)
image2 = cv2.rotate(image2, cv2.ROTATE_90_CLOCKWISE)

cos_theta = np.cos(-math.pi/2)
sin_theta = np.sin(-math.pi/2)
rotationMatrix = np.array(((cos_theta, -sin_theta), (sin_theta, cos_theta)))

while True:
    tags = detector.detect(image)

    for tag in tags:
        cameraToTagEstimate = detector.estimatePose(tag)

        tagCorners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]

        tagValues = TAG_DICTIONARY['tags'][tag.getId() - 1]['pose']
        tagTranslation = tagValues['translation']
        tagRotation = tagValues['rotation']['quaternion']
        tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
        tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
        tagPose = Pose3d(tagT3D, tagR3D)

        camRotation = cameraToTagEstimate.rotation()
        camInvRotation = cameraToTagEstimate.inverse().rotation()
        rotatedCameraToTagEstimate = Transform3d(cameraToTagEstimate.translation(), camRotation)

        wpiTranslation = CoordinateSystem.convert(cameraToTagEstimate.translation().rotateBy(camInvRotation),
                                                  CoordinateSystem.EDN(),
                                                  CoordinateSystem.NWU())
        estimatedRobotTransform = tagPose.transformBy(Transform3d(wpiTranslation, Rotation3d())).transformBy(
            cameraToRobotTransform)
        estimatedRobotPose = Pose3d(estimatedRobotTransform.translation(), camRotation)

        points = np.array([(int(p.x), int(p.y)) for p in tagCorners])

        targetDrawer = TargetDrawer(points)
        targetDrawer.drawTargetLines(image)
        targetDrawer.updateTargetPoints(points)

        targetDrawer.addText(image, "tag_id: {}".format(tag.getId()))
        targetDrawer.addText(image, "camTag-T: ({:.2f}, {:.2f} {:.2f})".format(
            rotatedCameraToTagEstimate.translation().x,
            rotatedCameraToTagEstimate.translation().y,
            rotatedCameraToTagEstimate.translation().z))
        targetDrawer.addText(image, "camTag-R: ({:.2f}, {:.2f} {:.2f})".format(
            rotatedCameraToTagEstimate.rotation().x_degrees,
            rotatedCameraToTagEstimate.rotation().y_degrees,
            rotatedCameraToTagEstimate.rotation().z_degrees))
        targetDrawer.addText(image, "eBotposeT: ({:.2f}, {:.2f}, {:.2f})".format(
            estimatedRobotPose.translation().x,
            estimatedRobotPose.translation().y,
            estimatedRobotPose.translation().z))
        targetDrawer.addText(image, "eBotposeR: ({:.2f}, {:.2f}, {:.2f})".format(
            estimatedRobotPose.rotation().x_degrees,
            estimatedRobotPose.rotation().y_degrees,
            estimatedRobotPose.rotation().z_degrees))
        targetDrawer.addText(image, "robotpose: ({:.2f}, {:.2f}, {:.2f})".format(
            estimatedRobotPose.translation().x,
            estimatedRobotPose.translation().y,
            estimatedRobotPose.rotation().y_degrees))

    tags = detector.detect(image2)

    for tag in tags:
        cameraToTagEstimate = detector.estimatePose(tag)

        tagCorners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]

        tagValues = TAG_DICTIONARY['tags'][tag.getId() - 1]['pose']
        tagTranslation = tagValues['translation']
        tagRotation = tagValues['rotation']['quaternion']
        tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
        tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
        tagPose = Pose3d(tagT3D, tagR3D)

        camRotation = cameraToTagEstimate.rotation()
        camInvRotation = cameraToTagEstimate.inverse().rotation()
        camRotation = camRotation.rotateBy(Rotation3d(0, 0, -math.pi/2))
        camInvRotation = camInvRotation.rotateBy(Rotation3d(0, 0, -math.pi/2))

        rotatedCameraToTagEstimate = Transform3d(cameraToTagEstimate.translation(), camRotation)

        wpiTranslation = CoordinateSystem.convert(cameraToTagEstimate.translation().rotateBy(camInvRotation),
                                                  CoordinateSystem.EDN(),
                                                  CoordinateSystem.NWU())
        estimatedRobotTransform = tagPose.transformBy(Transform3d(wpiTranslation, Rotation3d())).transformBy(
            cameraToRobotTransform)
        estimatedRobotPose = Pose3d(estimatedRobotTransform.translation(), camRotation)

        # points = np.array([(int(p.x), int(p.y)) for p in tagCorners])

        rotation = -math.pi / 2.0
        refPoint = Point(800, 600)

        # M_inv = cv2.getRotationMatrix2D((800 / 2, 600 / 2), 90, 1)
        # points = cv2.transform(points, M_inv)
        points = [cvUtils.rotatePoint(p, refPoint, rotation) for p in tagCorners]


        points = np.array([(int(p.x), int(p.y)) for p in points])

        targetDrawer = TargetDrawer(points)
        targetDrawer.drawTargetLines(image2)
        targetDrawer.updateTargetPoints(points)

        targetDrawer.addText(image2, "tag_id: {}".format(tag.getId()))
        targetDrawer.addText(image2, "camTag-T: ({:.2f}, {:.2f} {:.2f})".format(
            rotatedCameraToTagEstimate.translation().x,
            rotatedCameraToTagEstimate.translation().y,
            rotatedCameraToTagEstimate.translation().z))
        targetDrawer.addText(image2, "camTag-R: ({:.2f}, {:.2f} {:.2f})".format(
            rotatedCameraToTagEstimate.rotation().x_degrees,
            rotatedCameraToTagEstimate.rotation().y_degrees,
            rotatedCameraToTagEstimate.rotation().z_degrees))
        targetDrawer.addText(image2, "eBotposeT: ({:.2f}, {:.2f}, {:.2f})".format(
            estimatedRobotPose.translation().x,
            estimatedRobotPose.translation().y,
            estimatedRobotPose.translation().z))
        targetDrawer.addText(image2, "eBotposeR: ({:.2f}, {:.2f}, {:.2f})".format(
            estimatedRobotPose.rotation().x_degrees,
            estimatedRobotPose.rotation().y_degrees,
            estimatedRobotPose.rotation().z_degrees))
        targetDrawer.addText(image2, "robotpose: ({:.2f}, {:.2f}, {:.2f})".format(
            estimatedRobotPose.translation().x,
            estimatedRobotPose.translation().y,
            estimatedRobotPose.rotation().y_degrees))

    cv2.imshow("Image", image)
    cv2.imshow("CW_Image", image2)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

