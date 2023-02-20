import math

from wpimath import geometry
from wpimath import units
from wpimath.geometry import CoordinateAxis, CoordinateSystem, Transform3d, Translation3d, Rotation3d, Quaternion, \
    Pose3d

from aprilTags.tag_dictionary import TAG_DICTIONARY


tag_dictionary = TAG_DICTIONARY

# RobotPose = (12.192, 0, 45)
robotPose = Pose3d(Translation3d(units.inchesToMeters(480), 0, 0), Rotation3d(0, 0, math.radians(45)))
robotToCamera = Transform3d()

tagValues = tag_dictionary['tags'][0]['pose']
tagTranslation = tagValues['translation']
tagRotation = tagValues['rotation']['quaternion']
tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
tagPose = Pose3d(tagT3D, tagR3D)
tag1Estimate = Transform3d(Translation3d(units.inchesToMeters(62.635519), 0, units.inchesToMeters(122.301189)),
                           Rotation3d(math.radians(27.1188712), 0, 0))

wpiTranslation1 = CoordinateSystem.convert(tag1Estimate.translation(),
                                           CoordinateSystem.EDN(),
                                           CoordinateSystem.NWU())

rotatedTranslation = wpiTranslation1.rotateBy(tag1Estimate.rotation())


estimatedRobotPose = tagPose.transformBy(Transform3d(rotatedTranslation, Rotation3d()))\
    .transformBy(robotToCamera)

print("Estimated Pose: ({:.02f}, {:.02f}, {:.02f})".format(estimatedRobotPose.translation().x,
                                                           estimatedRobotPose.translation().y,
                                                           estimatedRobotPose.rotation().y_degrees))
print("Ideal Bot Pose: ({:.02f}, {:.02f}, {:.02f})".format(robotPose.translation().x,
                                                           robotPose.translation().y,
                                                           robotPose.rotation().y_degrees))

idealTranslation = Translation3d(units.inchesToMeters(130.77), units.inchesToMeters(42.19), 0)
idealTransform = idealTranslation.rotateBy(Rotation3d(0, 0, math.radians(-45)))
estimatedTagPose = robotPose.transformBy(robotToCamera).transformBy(Transform3d(idealTransform, Rotation3d()))

print("Tag Pose: ({:.02f}, {:.02f}, {:.02f})".format(tagPose.translation().x,
                                                           tagPose.translation().y,
                                                           tagPose.rotation().y_degrees))
print("Tag Pose from Bot: ({:.02f}, {:.02f}, {:.02f})".format(estimatedTagPose.translation().x,
                                                           estimatedTagPose.translation().y,
                                                           estimatedTagPose.rotation().y_degrees))

estimatedRobotPose2 = tagPose.transformBy(Transform3d(idealTranslation, Rotation3d(0, 0, math.radians(-45)))) \
    .transformBy(robotToCamera)

print("Bot Pose: ({:.02f}, {:.02f}, {:.02f})".format(robotPose.translation().x,
                                                           robotPose.translation().y,
                                                           robotPose.rotation().y_degrees))
print("Bot Pose from tag: ({:.02f}, {:.02f}, {:.02f})".format(estimatedRobotPose2.translation().x,
                                                           estimatedRobotPose2.translation().y,
                                                           estimatedRobotPose2.rotation().y_degrees))
