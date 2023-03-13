import math
import robotpy_apriltag
from wpimath.geometry import Rotation3d

from common import cvUtils

Point = robotpy_apriltag.AprilTagDetection.Point

centerPoint = Point(800, 600)
targetPoint = Point(800, 400)

rotatedPoint = cvUtils.rotatePoint(targetPoint, centerPoint, math.pi / 2)

print("New Point: ({}, {})".format(rotatedPoint.x, rotatedPoint.y))

rotation = Rotation3d(x=-0.018982, y=-0.043176, z=-1.585231)
invRotation = Rotation3d(x=0.042898, y=-0.019603, z=1.585221)

rotation.rotateBy(Rotation3d)
