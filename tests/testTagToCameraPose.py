from wpimath import geometry
from wpimath.geometry import CoordinateAxis, CoordinateSystem, Transform3d, Translation3d, Rotation3d, Quaternion, \
    Pose3d

from aprilTags.tag_dictionary import TAG_DICTIONARY


tag_dictionary = TAG_DICTIONARY

tagValues = tag_dictionary['tags'][1]['pose']
tagTranslation = tagValues['translation']
tagRotation = tagValues['rotation']['quaternion']
tagT3D = Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
tagR3D = Rotation3d(Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
tagPose = Pose3d(tagT3D, tagR3D)
robotToCamera = Transform3d()

tagTransform = Transform3d(Translation3d(-3, -2, -1), Rotation3d(0, 0, 0))

wpiTranslation = CoordinateSystem.convert(tagTransform.translation(),
                                          CoordinateSystem.EDN(),
                                          CoordinateSystem.NWU())

robotPose = tagPose.transformBy(Transform3d(wpiTranslation, tagTransform.rotation()).inverse()) \
                    .transformBy(robotToCamera)

print("Tag Position - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(tagPose.x, tagPose.y, tagPose.z, tagPose.rotation().z_degrees))
print("Tag Translation: {}".format(wpiTranslation))
print("Robot Position: {}".format(robotPose))

wpiTransform2 = CoordinateSystem.convert(tagTransform,
                                        CoordinateSystem.EDN(),
                                        CoordinateSystem.NWU())

robotPose2 = tagPose.transformBy(wpiTransform2.inverse())\
                    .transformBy(robotToCamera)

print("Robot Position 2: {}".format(robotPose2))


# print(geometry.Pose2d(geometry.Translation2d(0,1), geometry.Rotation2d(0)).transformBy(geometry.Transform2d(0,1,0)))
# print(geometry.Pose2d(geometry.Translation2d(0,1), geometry.Rotation2d(3.1416/2)).transformBy(geometry.Transform2d(0,1,0)))
# print(geometry.Pose2d(geometry.Translation2d(0,1), geometry.Rotation2d(3.1416/2)).transformBy(geometry.Transform2d(0,1,3.1416/2)))