from wpimath import geometry

from aprilTags.tag_dictionary import TAG_DICTIONARY


tag_dictionary = TAG_DICTIONARY

tagValues = tag_dictionary['tags'][1]['pose']
tagTranslation = tagValues['translation']
tagRotation = tagValues['rotation']['quaternion']
tagT3D = geometry.Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
tagR3D = geometry.Rotation3d(
    geometry.Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
tagPose = geometry.Pose3d(tagT3D, tagR3D)
robotToCamera = geometry.Transform3d()

tagTransform = geometry.Transform3d(geometry.Translation3d(1, 0, 1), geometry.Rotation3d(0, 0, 0))

wpiTransform = geometry.CoordinateSystem.convert(tagTransform, geometry.CoordinateSystem.EDN(), geometry.CoordinateSystem.NED())

wpiTransform2 = geometry.Transform3d(geometry.Translation3d(wpiTransform.x, -wpiTransform.y, wpiTransform.z), wpiTransform.rotation())

robotPose = tagPose.transformBy(wpiTransform2) \
    .transformBy(robotToCamera)

print("Tag Position - X: {}\tY: {}".format(tagPose.x, tagPose.y))
print("Robot Position - X: {}\tY: {}".format(robotPose.x, robotPose.y))
