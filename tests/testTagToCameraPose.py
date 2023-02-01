from wpimath import geometry
# from wpimath.geometry import CoordinateAxis, CoordinateSystem
#
# from aprilTags.tag_dictionary import TAG_DICTIONARY
#
#
# tag_dictionary = TAG_DICTIONARY
#
# tagValues = tag_dictionary['tags'][1]['pose']
# tagTranslation = tagValues['translation']
# tagRotation = tagValues['rotation']['quaternion']
# tagT3D = geometry.Translation3d(tagTranslation['x'], tagTranslation['y'], tagTranslation['z'])
# tagR3D = geometry.Rotation3d(
#     geometry.Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'], tagRotation['Z']))
# tagPose = geometry.Pose3d(tagT3D, tagR3D)
# robotToCamera = geometry.Transform3d()
#
# tagTransform = geometry.Transform3d(geometry.Translation3d(3, 0, -1), geometry.Rotation3d(0, 0, 0))
#
#
# cameraCoordinateSystem = CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.U(), CoordinateAxis.S())
# # tagTransform2 = geometry.Transform3d(geometry.Translation3d(tagTransform.x, -tagTransform.y, tagTransform.z), tagTransform.rotation())
# wpiTransform = CoordinateSystem.convert(tagTransform,
#                                         cameraCoordinateSystem,
#                                         CoordinateSystem.NWU())
#
# robotPose = tagPose.transformBy(wpiTransform.inverse())\
#                     .transformBy(robotToCamera)
#
# print("Tag Position - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(tagPose.x, tagPose.y, tagPose.z, tagPose.rotation().z_degrees))
# print("Tag Translation - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(wpiTransform.inverse().x, wpiTransform.inverse().y, wpiTransform.inverse().z, wpiTransform.inverse().rotation().z_degrees))
# print("Robot Position - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(robotPose.x, robotPose.y, robotPose.z, robotPose.rotation().z_degrees))
#
# wpiTransform2 = CoordinateSystem.convert(tagTransform,
#                                         CoordinateSystem.EDN(),
#                                         CoordinateSystem.NWU())
#
# robotPose2 = tagPose.transformBy(wpiTransform2.inverse())\
#                     .transformBy(robotToCamera)
#
# print("Robot Position2 - X: {:.02f}\tY: {:.02f}\tZ: {:.02f}\tR: {:.02f}".format(robotPose2.x, robotPose2.y, robotPose2.z, robotPose2.rotation().z_degrees))
#
#
print(geometry.Pose2d(geometry.Translation2d(0,1), geometry.Rotation2d(0)).transformBy(geometry.Transform2d(0,1,0)))
print(geometry.Pose2d(geometry.Translation2d(0,1), geometry.Rotation2d(3.1416/2)).transformBy(geometry.Transform2d(0,1,0)))
print(geometry.Pose2d(geometry.Translation2d(0,1), geometry.Rotation2d(3.1416/2)).transformBy(geometry.Transform2d(0,1,3.1416/2)))