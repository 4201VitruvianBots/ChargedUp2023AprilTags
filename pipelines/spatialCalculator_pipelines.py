import copy
import logging

import depthai as dai

from common import constants

log = logging.getLogger(__name__)


def create_stereoDepth_pipeline(enable_recording=False):
    global pipeline
    global pipeline_info

    # Start defining a pipeline
    pipeline = dai.Pipeline()

    pipeline.setXLinkChunkSize(0)

    # Define a source - two mono (grayscale) cameras
    monoLeft = pipeline.createMonoCamera()
    monoRight = pipeline.createMonoCamera()
    stereo = pipeline.createStereoDepth()
    xoutDepth = pipeline.createXLinkOut()
    xoutRight = pipeline.createXLinkOut()
    xinRight = pipeline.createXLinkIn()

    depthStr = "depth"
    monoRightStr = "right"
    monoRightCtrlStr = "monoRightCtrl"

    xoutDepth.setStreamName(depthStr)
    xoutRight.setStreamName(monoRightStr)
    xinRight.setStreamName(monoRightCtrlStr)

    # MonoCamera
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoLeft.setFps(120)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoRight.setFps(120)
    monoRight.out.link(xoutRight.input)
    # Exposure time (microseconds), ISO Sensitivity (100-1600)
    monoRight.initialControl.setManualExposure(3000, 200)
    # Temperature in Kelvins (1000-12000)
    monoRight.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
    monoRight.initialControl.setManualWhiteBalance(6000)
    monoRight.initialControl.setBrightness(5)

    outputDepth = True
    outputRectified = False
    lrcheck = False
    subpixel = False

    # StereoDepth
    stereo.setOutputDepth(outputDepth)
    stereo.setOutputRectified(outputRectified)
    stereo.setDepthAlign(dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)
    stereo.setConfidenceThreshold(255)
    stereo.setRectifyEdgeFillColor(0)
    stereo.setMedianFilter(dai.MedianFilter.KERNEL_7x7)

    stereo.setLeftRightCheck(lrcheck)
    stereo.setSubpixel(subpixel)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    stereo.depth.link(xoutDepth.input)
    xinRight.out.link(monoRight.inputControl)

    encoderStr = "h265"
    if enable_recording:
        videoEncoder = pipeline.createVideoEncoder()
        videoEncoder.setDefaultProfilePreset(120, dai.VideoEncoderProperties.Profile.H264_HIGH)
        videoEncoder.setLossless(True)
        videoOut = pipeline.create(dai.node.XLinkOut)
        videoOut.setStreamName(encoderStr)
        monoRight.out.link(videoEncoder.input)
        videoEncoder.bitstream.link(videoOut.input)

    pipeline_info = {
        'resolution_x': monoRight.getResolutionWidth(),
        'resolution_y': monoRight.getResolutionHeight(),
        'depthQueue': depthStr,
        'monoRightQueue': monoRightStr,
        'encoderQueue': encoderStr,
        'monoRightCtrlQueue': monoRightCtrlStr
        # 'imuQueue': imuQueueStr
    }

    return pipeline, pipeline_info


def create_spatialCalculator_pipeline():
    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Define a source - two mono (grayscale) cameras
    monoLeft = pipeline.createMonoCamera()
    monoRight = pipeline.createMonoCamera()
    stereo = pipeline.createStereoDepth()
    spatialLocationCalculator = pipeline.createSpatialLocationCalculator()

    xoutDepth = pipeline.createXLinkOut()
    xoutSpatialData = pipeline.createXLinkOut()
    xinSpatialCalcConfig = pipeline.createXLinkIn()
    xoutRight = pipeline.createXLinkOut()

    depthStr = "depth"
    monoRightStr = "right"
    spatialDataStr = "spatialData"
    spatialCalcConfigStr = "spatialCalcConfig"

    xoutDepth.setStreamName(depthStr)
    xoutSpatialData.setStreamName(spatialDataStr)
    xinSpatialCalcConfig.setStreamName(spatialCalcConfigStr)
    xoutRight.setStreamName(monoRightStr)

    # MonoCamera
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoLeft.setFps(120)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoRight.setFps(120)
    monoRight.out.link(xoutRight.input)

    outputDepth = True
    outputRectified = False
    lrcheck = False
    subpixel = False

    # StereoDepth
    stereo.setOutputDepth(outputDepth)
    stereo.setOutputRectified(outputRectified)
    stereo.setDepthAlign(dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)
    stereo.setConfidenceThreshold(255)
    stereo.setRectifyEdgeFillColor(0)
    stereo.setMedianFilter(dai.MedianFilter.KERNEL_7x7)

    stereo.setLeftRightCheck(lrcheck)
    stereo.setSubpixel(subpixel)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
    stereo.depth.link(spatialLocationCalculator.inputDepth)

    spatialLocationCalculator.setWaitForConfigInput(False)
    config = dai.SpatialLocationCalculatorConfigData()
    config.depthThresholds.lowerThreshold = 100
    config.depthThresholds.upperThreshold = 10000
    topLeft = dai.Point2f(0.4, 0.4)
    bottomRight = dai.Point2f(0.6, 0.6)
    config.roi = dai.Rect(topLeft, bottomRight)
    spatialLocationCalculator.initialConfig.addROI(config)
    spatialLocationCalculator.out.link(xoutSpatialData.input)
    xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

    pipeline_info = {
        'resolution_x': monoRight.getResolutionWidth(),
        'resolution_y': monoRight.getResolutionHeight(),
        'depthQueue': depthStr,
        'monoRightQueue': monoRightStr,
        'spatialDataQueue': spatialDataStr,
        'spatialCalcConfigQueue': spatialCalcConfigStr
    }

    return pipeline, pipeline_info


def capture(device_info, camera_settings=None):
    with dai.Device(pipeline, device_info) as device:
        # device.setIrLaserDotProjectorBrightness(200)
        # device.setIrFloodLightBrightness(1500)

        depthQueue = device.getOutputQueue(name=pipeline_info["depthQueue"], maxSize=1, blocking=False)
        qRight = device.getOutputQueue(name=pipeline_info["monoRightQueue"], maxSize=1, blocking=False)
        qInputRight = device.getInputQueue(pipeline_info["monoRightCtrlQueue"])

        if camera_settings is not None:
            prev_camera_settings = copy.copy(camera_settings)
        while True:
            try:
                inDepth = depthQueue.get()  # blocking call, will wait until a new data has arrived
                inRight = qRight.get()
            except Exception as e:
                log.error("Frame not received")
                continue

            depthFrame = inDepth.getFrame()
            frameRight = inRight.getCvFrame()  # get mono right frame
            timestampNs = inRight.getTimestamp()

            if camera_settings is not None:
                if prev_camera_settings != camera_settings:
                    ctrl = dai.CameraControl()
                    ctrl.setManualExposure(camera_settings['manual_exposure_usec'], camera_settings['manual_exposure_iso'])
                    ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
                    ctrl.setManualWhiteBalance(camera_settings['white_balance'])
                    ctrl.setBrightness(camera_settings['brightness'])
                    qInputRight.send(ctrl)
                    prev_camera_settings = copy.copy(camera_settings)

            yield frameRight, depthFrame, timestampNs