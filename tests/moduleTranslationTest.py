
from wpimath.geometry import Translation2d, Pose2d, Rotation2d, Transform2d

driveWidth = 0.5
driveLength = 0.5

moduleTranslations = [
    Translation2d(driveWidth, driveLength),
    Translation2d(driveWidth, -driveLength),
    Translation2d(-driveWidth, driveLength),
    Translation2d(-driveWidth, -driveLength),
]

robotPose = Pose2d(5, 5, Rotation2d.fromDegrees(45))

modulePoses = []
for module in moduleTranslations:
    moduleTransform = Transform2d(module, Rotation2d.fromDegrees(0))
    moduleTranslation = robotPose.transformBy(moduleTransform)

    modulePoses.append(Pose2d(moduleTranslation.translation(), robotPose.rotation().rotateBy(Rotation2d.fromDegrees(0))))


def m2in(m):
    return m * 39.3701

# (196.850394, 210.769819) (A, B)
# (210.769819, 196.850394) (B, A)
# (182.930969, 196.850394) (C, A)
# (196.850394, 182.930969) (A, C)


for modulePose in modulePoses:
    print("({:.02f}, {:.02f}, {})".format(m2in(modulePose.translation().x),
                                          m2in(modulePose.translation().y),
                                          modulePose.rotation().degrees()))


