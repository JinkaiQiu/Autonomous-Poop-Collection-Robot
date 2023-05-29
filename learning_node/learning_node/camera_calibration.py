import depthai as dai

# Start pipeline
pipeline = dai.Pipeline()

# Define source - rgb camera
rgb = pipeline.createColorCamera()
rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

# Create output
xout = pipeline.createXLinkOut()
xout.setStreamName('rgb')

# Connect camera to output
rgb.preview.link(xout.input)

# Pipeline defined, now the device is assigned and pipeline is started
with dai.Device(pipeline) as device:
    # Get RGB camera intrinsics
    rgbQueue = device.getOutputQueue('rgb', maxSize=1, blocking=False)
    rgbCameraCalib = device.readCalibration().getCameraIntrinsics(dai.CameraBoardSocket.RGB)

    print("RGB camera intrinsics:")
    print(f"fx: {rgbCameraCalib[0][0]}")
    print(f"fy: {rgbCameraCalib[1][1]}")
    print(f"cx: {rgbCameraCalib[0][2]}")
    print(f"cy: {rgbCameraCalib[1][2]}")
