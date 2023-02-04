import copy
import logging

import depthai as dai

log = logging.getLogger(__name__)


def create_hires_apriltag_pipeline():
    global pipeline
    global pipeline_info

    # Create pipeline
    pipeline = dai.Pipeline()

    pipeline.setXLinkChunkSize(0)
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    manip = pipeline.create(dai.node.ImageManip)
    xOut = pipeline.createXLinkOut()

    xOut.setStreamName("monoRgb")
    # Properties
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    # camRgb.initialControl.setManualExposure(6000, 200)
    # Temperature in Kelvins (1000-12000)
    camRgb.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
    # camRgb.initialControl.setManualWhiteBalance(6000)
    # camRgb.initialControl.setBrightness(5)

    manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
    manip.setMaxOutputFrameSize(8294400)

    # Linking
    camRgb.video.link(manip.inputImage)
    manip.out.link(xOut.input)

    return pipeline


def create_mono_apriltag_pipeline():
    global pipeline
    global pipeline_info

    # Create pipeline
    pipeline = dai.Pipeline()

    pipeline.setXLinkChunkSize(0)
    # Define sources and outputs
    camMono = pipeline.create(dai.node.MonoCamera)
    xOut = pipeline.createXLinkOut()

    xOut.setStreamName("monoRgb")
    # Properties
    camMono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    camMono.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    camMono.initialControl.setManualExposure(3000, 200)
    # Temperature in Kelvins (1000-12000)
    camMono.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
    camMono.initialControl.setManualWhiteBalance(6000)
    camMono.initialControl.setBrightness(5)

    # Linking
    camMono.out.link(xOut.input)

    return pipeline


def create_dual_mono_apriltag_pipeline():
    global pipeline
    global pipeline_info

    # Create pipeline
    pipeline = dai.Pipeline()

    pipeline.setXLinkChunkSize(0)
    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    xOutLeft = pipeline.createXLinkOut()
    xOutRight = pipeline.createXLinkOut()

    monoLeftStr = "monoLeft"
    monoRightStr = "monoRight"
    xOutLeft.setStreamName(monoLeftStr)
    xOutRight.setStreamName(monoRightStr)

    # Properties
    for monoCam in [monoLeft, monoRight]:
        monoCam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        monoCam.setFps(120)

        monoCam.initialControl.setManualExposure(3000, 200)
        # Temperature in Kelvins (1000-12000)
        monoCam.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
        monoCam.initialControl.setManualWhiteBalance(6000)
        monoCam.initialControl.setBrightness(5)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # Linking
    monoLeft.out.link(xOutLeft.input)
    monoRight.out.link(xOutRight.input)

    pipeline_info = {
        'resolution_x': monoRight.getResolutionWidth(),
        'resolution_y': monoRight.getResolutionHeight(),
        'monoLeftQueue': monoLeftStr,
        'monoRightQueue': monoRightStr,
    }

    return pipeline, pipeline_info


def dual_capture(device_info, camera_settings=None):
    errorCount = 0
    with dai.Device(pipeline, device_info) as device:
        # device.setIrLaserDotProjectorBrightness(200)
        # device.setIrFloodLightBrightness(1500)

        qLeft = device.getOutputQueue(name=pipeline_info["monoLeftQueue"], maxSize=1, blocking=False)
        qRight = device.getOutputQueue(name=pipeline_info["monoRightQueue"], maxSize=1, blocking=False)
        # qInputRight = device.getInputQueue(pipeline_info["monoRightCtrlQueue"])

        if camera_settings is not None:
            prev_camera_settings = copy.copy(camera_settings)
        while True:
            try:
                inLeft = qLeft.get()  # blocking call, will wait until a new data has arrived
                inRight = qRight.get()
                errorCount = 0
            except Exception as e:
                log.error("Frame not received")
                errorCount = errorCount + 1
                if errorCount > 5:
                    log.error("Too many frames dropped. Attempting to restart")
                    yield -1, None, None, None
                continue

            frameLeft = inLeft.getFrame()
            frameRight = inRight.getCvFrame()  # get mono right frame
            timestampLns = inLeft.getTimestamp()
            timestampRns = inRight.getTimestamp()

            # if camera_settings is not None:
            #     if prev_camera_settings != camera_settings:
            #         ctrl = dai.CameraControl()
            #         ctrl.setManualExposure(camera_settings['manual_exposure_usec'], camera_settings['manual_exposure_iso'])
            #         ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
            #         ctrl.setManualWhiteBalance(camera_settings['white_balance'])
            #         ctrl.setBrightness(camera_settings['brightness'])
            #         qInputRight.send(ctrl)
            #         prev_camera_settings = copy.copy(camera_settings)

            yield 0, frameLeft, frameRight, timestampLns, timestampRns