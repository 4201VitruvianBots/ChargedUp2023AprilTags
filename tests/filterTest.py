from wpimath.geometry import Transform3d, Translation3d, Rotation3d
import math
from wpimath.units import inchesToMeters
tagTranslation = Transform3d(Translation3d(-0.315440, -0.160931, 0.464444), Rotation3d(0.046023, -0.308639, -0.249401))
angleThreshold = 5

if tagTranslation.rotation().x_degrees < angleThreshold:
    print("tag has invalid angle")
if tagTranslation.rotation().y_degrees < angleThreshold:
    print("tag has invalid angle")
if tagTranslation.rotation().z_degrees < angleThreshold:
    print("tag has invalid angle")


tagDistance = tagTranslation.translation().norm()
print(tagDistance)

if tagDistance > inchesToMeters(90):
    print("tag has invalid distance")