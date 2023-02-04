import cscore

for caminfo in cscore.UsbCamera.enumerateUsbCameras():
    print("%s: %s (%s)" % (caminfo.dev, caminfo.path, caminfo.name))
    if caminfo.otherPaths:
        print("Other device paths:")
        for path in caminfo.otherPaths:
            print(" ", path)

    camera = cscore.VideoSource()