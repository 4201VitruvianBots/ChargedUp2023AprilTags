import cv2
import numpy as np

from common import constants


class TargetDrawer:

    def __init__(self, targetPoints, fontSize=0.6, fontColor=(255, 255, 255)):
        self.targetPoints = targetPoints
        self.fontSize = fontSize
        self.fontIncrement = int(self.fontSize * 30)
        self.textX = max(self.targetPoints[:, 0])
        self.textY = min(self.targetPoints[:, 1])
        self.fontColor = fontColor

    def drawTargetLines(self, frame, lineColor=(120, 120, 120), lineThickness=3):
        cv2.polylines(frame, [self.targetPoints], True, lineColor, lineThickness)

    def drawTargetBox(self, frame, iMatrix, boxTranslation, lineColor=(0, 255, 0), lineThickness=1):
        r_vec = np.array([boxTranslation.rotation().x,
                          boxTranslation.rotation().y,
                          boxTranslation.rotation().z])
        t_vec = np.array([boxTranslation.translation().x,
                          boxTranslation.translation().y,
                          boxTranslation.translation().z])

        ipoints, _ = cv2.projectPoints(constants.OPOINTS,
                                       r_vec,
                                       t_vec,
                                       iMatrix,
                                       np.zeros(5))

        ipoints = np.round(ipoints).astype(int)

        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

        for i, j in constants.EDGES:
            cv2.line(frame, ipoints[i], ipoints[j], lineColor, lineThickness, cv2.LINE_AA)

    def addText(self, frame, text, textThickness=1):
        # Draw white text over black text to make it easier to read
        cv2.putText(frame, text, (self.textX, self.textY), cv2.FONT_HERSHEY_TRIPLEX, self.fontSize, (255, 255, 255), textThickness, 5)
        cv2.putText(frame, text, (self.textX, self.textY), cv2.FONT_HERSHEY_TRIPLEX, self.fontSize, self.fontColor, textThickness, 2)
        self.textY += self.fontIncrement


def drawStats(frame, stats, fontSize=1, fontColor=(255, 255, 255)):
    fontIncrement = int(fontSize * 25)
    yValue = fontSize * fontIncrement
    cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 1, fontColor, 1)

    for stat in stats:
        cv2.putText(frame, stat, (0, yValue), cv2.FONT_HERSHEY_TRIPLEX, fontSize, fontColor)
        yValue += fontIncrement