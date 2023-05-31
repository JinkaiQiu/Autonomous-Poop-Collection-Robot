import depthai as dai

with dai.Device() as device:
  calibData = device.readCalibration()
  intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
  print('RGB camera focal length fx in pixels:', intrinsics[0][0])
  print('RGB camera focal length fy in pixels:', intrinsics[1][1])
  print('RGB camera center cx in pixels:', intrinsics[0][2])
  print('RGB camera center cy in pixels:', intrinsics[1][2])

  rgbCameraCalib = device.readCalibration().getCameraIntrinsics(dai.CameraBoardSocket.RGB)

  print("RGB camera intrinsics:")
  print(f"fx: {rgbCameraCalib[0][0]}")
  print(f"fy: {rgbCameraCalib[1][1]}")
  print(f"cx: {rgbCameraCalib[0][2]}")
  print(f"cy: {rgbCameraCalib[1][2]}")
  

# # First unofficial data.
# import depthai as dai

# # Start pipeline
# pipeline = dai.Pipeline()

# # Define source - rgb camera
# rgb = pipeline.createColorCamera()
# rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

# # Create output
# xout = pipeline.createXLinkOut()
# xout.setStreamName('rgb')

# # Connect camera to output
# rgb.preview.link(xout.input)

# # Pipeline defined, now the device is assigned and pipeline is started
# with dai.Device(pipeline) as device:
    # Get RGB camera intrinsics
    # rgbQueue = device.getOutputQueue('rgb', maxSize=1, blocking=False)
    # rgbCameraCalib = device.readCalibration().getCameraIntrinsics(dai.CameraBoardSocket.RGB)

    # print("RGB camera intrinsics:")
    # print(f"fx: {rgbCameraCalib[0][0]}")
    # print(f"fy: {rgbCameraCalib[1][1]}")
    # print(f"cx: {rgbCameraCalib[0][2]}")
    # print(f"cy: {rgbCameraCalib[1][2]}")
