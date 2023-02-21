import math

from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath import units

from aprilTags.tag_dictionary import TAG_DICTIONARY

tag_dictionary = TAG_DICTIONARY

tagPoses = []
for i in range(0, 3):
    tag1Translation = tag_dictionary['tags'][i]['pose']['translation']

    tagPoses.append(Pose2d(tag1Translation['x'], tag1Translation['y'], Rotation2d()))

intakeOffset = units.inchesToMeters(5)

robotPose = Pose2d(5, 5, Rotation2d())

nearestPose = robotPose.nearest(tagPoses)

solution = nearestPose.relativeTo(robotPose)

solutionTransform = Transform2d(Translation2d(0, intakeOffset), Rotation2d.fromDegrees(90))

correctSolution = solution.transformBy(solutionTransform)

difference = correctSolution.relativeTo(solution)
difference.translation().norm()
difference.rotation().degrees()

angleSetpoint = solution.translation().angle().degrees()

elevatorHorizontalSetpoint = solution.translation().norm()

print()
