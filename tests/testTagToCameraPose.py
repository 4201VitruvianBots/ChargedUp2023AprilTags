import math

from wpimath import geometry
from wpimath.geometry import CoordinateAxis, CoordinateSystem, Transform3d, Translation3d, Rotation3d, Quaternion, \
    Pose3d

from aprilTags.tag_dictionary import TAG_DICTIONARY


tag_dictionary = TAG_DICTIONARY

tagsToTest = [1, 2]
detectionResult = [
    Transform3d(Translation3d(-0.47, 0.04, 2.33), Rotation3d(math.radians(1.35), math.radians(22.20), math.radians(0.90))),
    Transform3d(Translation3d(-2.03, 0.00, 2.90), Rotation3d(math.radians(0.48), math.radians(19.35), math.radians(0.84))),
]


# RobotPose = (12.192, 0, 45)
robotToCamera = Transform3d(Translation3d(), Rotation3d(0, 0, 0))
for tag in tagsToTest:
    tagValues = tag_dictionary['tags'][tag - 1]['pose']
    tagTranslation = tagValues['translation']
    tagRotation = tagValues['rotation']['quaternion']
    tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
    tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
    tagPose = Pose3d(tagT3D, tagR3D)

    tagTransform = detectionResult[tag - 1]
    wpiTranslation = CoordinateSystem.convert(tagTransform.translation().rotateBy(tagTransform.inverse().rotation()),
                                              CoordinateSystem.EDN(),
                                              CoordinateSystem.NWU())
    # wpiTranslation = wpiTranslation.rotateBy(tagTransform.rotation())
    robotPose = tagPose.transformBy(Transform3d(wpiTranslation, Rotation3d())).transformBy(robotToCamera)

    print("Tag Position - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(tagPose.x, tagPose.y, tagPose.z, tagPose.rotation().z_degrees))
    print("Tag Translation: {}".format(wpiTranslation))
    print("robotpose: ({:.02f}, {:.02f}, {:.02f})".format(robotPose.translation().x, robotPose.translation().y, robotPose.rotation().y_degrees))

# tagTransform2 = Transform3d(Translation3d(-0.78, 0, 1.82), Rotation3d(math.radians(-180), math.radians(45), math.radians(0)))
# wpiTranslation2 = CoordinateSystem.convert(tagTransform2.translation(),
#                                           CoordinateSystem.EDN(),
#                                           CoordinateSystem.NWU())
#
# robotPose2 = tagPose.transformBy(Transform3d(wpiTranslation2, tagTransform2.rotation())) \
#                     .transformBy(robotToCamera)
#
# print("Tag Translation2: {}".format(wpiTranslation))
# print("robotpose2: ({:.02f}, {:.02f}, {:.02f})".format(robotPose2.translation().x, robotPose2.translation().y, robotPose2.rotation().y_degrees))