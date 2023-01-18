import depthai as dai


def create_hires_apriltag_pipeline():
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