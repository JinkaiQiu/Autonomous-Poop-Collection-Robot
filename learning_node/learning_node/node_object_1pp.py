# node_object_1pp.py

# Description:
# Detect the poop with the color filter
# Get and transform the 3D coordinate
# Publish the detected coordinates to the /poop_coordinates topic as PointStamped messages. 

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Check list: 
# 1. test the accuracy using rgb and infrared as the source frame. The necessary to use the manual calibration data
# 2. link offset: https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Lectures/TF%20(transform)%20in%20ROS.pdf
# 2. dno't need to draw the contour later

import depthai as dai
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped

from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_geometry_msgs

lower_brown = np.array([1, 125, 80])    # Poop的HSV阈值下限
upper_brown = np.array([10, 231, 254])  # Poop的HSV阈值上限

class PoopDetectorNode(Node):
    def __init__(self):
        super().__init__("poop_detector_node")
        self.bridge = CvBridge()
    
        # Depthai pipeline
        self.pipeline = dai.Pipeline()

        # Create a color camera node and configure it
        self.cam_rgb = self.pipeline.createColorCamera()
        self.cam_rgb.setPreviewSize(1280, 720)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setFps(30)

        # Create and configure a stereo depth node
        self.stereo = self.pipeline.createStereoDepth()
        self.stereo.setConfidenceThreshold(200)
        
        # Add this line to get the rectified frames which are aligned with the RGB frame
        # self.stereo.setOutputRectified(True)

        # Create and configure the mono (LEFT and RIGHT) cameras
        self.left = self.pipeline.createMonoCamera()
        self.left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.right = self.pipeline.createMonoCamera()
        self.right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Link the mono camera outputs to the stereo depth node
        self.left.out.link(self.stereo.left)
        self.right.out.link(self.stereo.right)

        # Create output nodes for the RGB camera and stereo depth node
        self.xout_rgb = self.pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(self.xout_rgb.input)
        self.xout_depth = self.pipeline.createXLinkOut()
        self.xout_depth.setStreamName("depth")
        self.stereo.depth.link(self.xout_depth.input)

        # Start the pipeline and get the device
        self.device = dai.Device(self.pipeline)

        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.point_publisher = self.create_publisher(PointStamped, '/poop_coordinates', 10) # create a publisher for 'Point Stamped message'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer callback at a rate of 1 Hz


    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()
        in_depth = self.q_depth.tryGet()
        if in_rgb is not None and in_depth is not None:
            frame = in_rgb.getCvFrame()
            depth_frame = in_depth.getCvFrame()
            depth_frame = cv2.resize(depth_frame, (frame.shape[1], frame.shape[0]))
            bounding_box = self.find_object(frame, depth_frame)
            if bounding_box is not None:
                self.convert_to_world_coordinates(depth_frame, bounding_box)
            else:
                self.get_logger().info("No object found in image")



    def find_object(self, image, depth_image):
        max_area = 0
        max_cnt = None
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        mask_red = cv2.inRange(hsv_img, lower_brown, upper_brown)
        contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for cnt in contours:
            if cnt.shape[0] < 5: 
                continue

            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_cnt = cnt

        if max_cnt is not None:
            (x, y, w, h) = cv2.boundingRect(max_cnt)
            cv2.drawContours(image, [max_cnt], -1, (0, 255, 0), 2)
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)

            cv2.imshow("object", image)
            #cv2.imshow("Depth object", depth_image)
            cv2.waitKey(500)
            return x, y, w, h

        return None

    def convert_to_world_coordinates(self, depth_image, bounding_box):
        # OAK-D RGB camera parameters (replace these with actual values)
        cx = 3075.462646484375  # principal point 
        cy = 3075.462646484375  # principal point 
        fx = 1919.828857421875  # focal length
        fy = 1079.656005859375  # focal length

        if bounding_box is not None and depth_image is not None:
            (x, y, w, h) = bounding_box

            # read the depth data (in millimeters)
            if int(y+h/2) < depth_image.shape[0] and int(x+w/2) < depth_image.shape[1]:
                depth = depth_image[int(y+h/2), int(x+w/2)]
                self.get_logger().info('Depth: ' + str(depth))
            else:
                self.get_logger().info('Bounding box center is out of image boundaries, ignoring...')
                return

            # check if the depth reading is a valid number (not zero)
            if depth == 0:
                self.get_logger().info('Invalid depth reading as 0, ignoring...')
                return

            # convert to meters
            depth = depth / 1000.0

            # convert to world coordinates
            u = int(x+w/2)
            v = int(y+h/2)
            world_X = (u - cx) * depth / fx
            world_Y = (v - cy) * depth / fy
            world_Z = depth

            # create a PointStamped message
            point_camera_frame = PointStamped()
            point_camera_frame.header.stamp = self.get_clock().now().to_msg()
            point_camera_frame.header.frame_id = 'oak_d_link'  # replace with your OAK-D frame ID
            point_camera_frame.point.x = world_X
            point_camera_frame.point.y = world_Y
            point_camera_frame.point.z = world_Z

            # transform the point to the map frame
            try:
                transform = self.tf_buffer.lookup_transform('map',  # target frame
                                            'oak_d_link',  # source frame
                                            rclpy.time.Time(),  # time
                                            rclpy.duration.Duration(seconds=5.0))  # timeout
                point_map_frame = tf2_geometry_msgs.do_transform_point(point_camera_frame, transform)
                self.point_publisher.publish(point_map_frame)  # publish the point
                self.get_logger().info('Publishing: "%s"' % point_map_frame.point) 
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                self.get_logger().warn('Transform lookup failed: %s' % ex)

def main(args=None):
   rclpy.init(args=args)
   poop_detector_node = PoopDetectorNode()
   rclpy.spin(poop_detector_node)

   poop_detector_node.destroy_node()
   rclpy.shutdown()


if __name__ == "__main__":
   main()
   
