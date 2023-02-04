import cv2
import depthai as dai
import logging

import numpy as np

from common.utils import FPSHandler
from cscore_utils.CSCoreServer import CSCoreServer
from pipelines import spatialCalculator_pipelines

log = logging.getLogger(__name__)
c_handler = logging.StreamHandler()
log.addHandler(c_handler)
log.setLevel(logging.INFO)

testServer = CSCoreServer("test", width=640, height=360, fps=30)

pipeline, pipeline_info = spatialCalculator_pipelines.create_stereoDepth_pipeline()

device_info = dai.DeviceInfo("169.254.1.222")
fps = FPSHandler()
latency = np.array([])

with dai.Device(pipeline, device_info) as device:
    # device.setIrLaserDotProjectorBrightness(200)
    # device.setIrFloodLightBrightness(1500)

    depthQueue = device.getOutputQueue(name=pipeline_info["depthQueue"], maxSize=1, blocking=False)
    qRight = device.getOutputQueue(name=pipeline_info["monoRightQueue"], maxSize=1, blocking=False)
    qInputRight = device.getInputQueue(pipeline_info["monoRightCtrlQueue"])

    while True:
        try:
            inDepth = depthQueue.get()  # blocking call, will wait until a new data has arrived
            inRight = qRight.get()
            fps.nextIter()
        except Exception as e:
            log.error("Frame not received")
            continue

        depthFrame = inDepth.getFrame()
        frameRight = inRight.getCvFrame()  # get mono right frame
        timestampNs = inRight.getTimestamp()

        fpsValue = fps.fps()
        latencyMs = (dai.Clock.now() - timestampNs).total_seconds() * 1000.0
        latency = np.append(latency, latencyMs)
        avgLatency = np.average(latency) if len(latency) < 100 else np.average(latency[-100:])
        latencyStd = np.std(latency) if len(latency) < 100 else np.std(latency[-100:])

        cv2.circle(frameRight, (int(frameRight.shape[1] / 2), int(frameRight.shape[0] / 2)), 1, (255, 255, 255), 1)
        cv2.putText(frameRight, "FPS: {:.2f}".format(fpsValue), (0, 24), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))
        cv2.putText(frameRight, "Latency: {:.2f}ms".format(avgLatency), (0, 50), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255))

        testServer.setFrame(frameRight)
        print("FPS: {}\tLatency: {}".format(fpsValue, avgLatency))
