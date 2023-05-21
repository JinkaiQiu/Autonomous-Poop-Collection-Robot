# node_object_1pp.py


# Depth coordinate without the shape filter only the color filter

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Check list: test the accuracy using rgb and infrared as the source frame

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import necessary modules
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from rclpy.time import Time
from rclpy.duration import Duration


import cv2                              # OpenCV图像处理库
import numpy as np                      # Python数值计算库

# RGB Camera Parameters
rgb_focal_length = np.array([1066.0224, 1065.5055])
rgb_principal_point = np.array([964.0866, 543.1938])
rgb_radial_distortion = np.array([0.0609, -0.0395])

# Infrared Camera Parameters
infrared_focal_length = np.array([365.5223, 364.8092])
infrared_principal_point = np.array([253.6649, 210.8401])
infrared_radial_distortion = np.array([0.0940, -0.2381])

# Construct distortion coefficient matrix
rgb_distortion_coefficients = np.array([rgb_radial_distortion[0],
                                        rgb_radial_distortion[1],
                                        0, 0, 0])
infrared_distortion_coefficients = np.array([infrared_radial_distortion[0],
                                        infrared_radial_distortion[1],
                                        0, 0, 0])

# Camera Matrix
rgb_camera_matrix = np.array([[rgb_focal_length[0], 0, rgb_principal_point[0]],
                              [0, rgb_focal_length[1], rgb_principal_point[1]],
                              [0, 0, 1]])

infrared_camera_matrix = np.array([[infrared_focal_length[0], 0, infrared_principal_point[0]],
                                   [0, infrared_focal_length[1], infrared_principal_point[1]],
                                   [0, 0, 1]])

#Color Range1 -- more saturated and brighter
lower_brown = np.array([10, 50, 60])    # Poop的HSV阈值下限
upper_brown = np.array([40, 225, 255])  # Poop的HSV阈值上限

#Color Range2 -- include more reddish or yellowish colors
# lower_brown = np.array([5, 30, 40])    # Poop的HSV阈值下限
# upper_brown = np.array([50, 200, 230])  # Poop的HSV阈值上限


class PoopDetectorNode(Node):
    def __init__(self):
        super().__init__("poop_detector_node")
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image,"/kinect2/qhd/image_color_rect", self.image_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, "/kinect2/qhd/image_depth_rect", self.depth_callback, 10)
        self.depth_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.undistort_image(cv_image, rgb_camera_matrix, rgb_distortion_coefficients)
        except CvBridgeError as e:
            print(e)
            return
        
        if self.depth_image is None:
            self.get_logger().warn('Depth image is None. Cannot find object.')
            return
        
        bounding_box = self.find_object(cv_image)
        if bounding_box is None:
            self.get_logger().warn('No bounding box found. The object might not be in the scene.')
            return

        world_coords = self.convert_to_world_coordinates(self.depth_image, bounding_box)
        if world_coords is None:
            self.get_logger().warn('Failed to convert to world coordinates. Check your transformation.')
            return

        self.get_logger().info(f"World Coordinates: {world_coords}")


    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_image = self.undistort_image(self.depth_image, infrared_camera_matrix, infrared_distortion_coefficients)
        except CvBridgeError as e:
            print(e)
            return
    
    def undistort_image(self, image, camera_matrix, distortion_coefficients):
        undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients)
        return undistorted_image

    def find_object(self, image):
        max_area = 0
        max_cnt = None
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        mask_red = cv2.inRange(hsv_img, lower_brown, upper_brown)
        contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for cnt in contours:
            if cnt.shape[0] < 150:
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
            cv2.waitKey(500)
            return x, y, w, h

        return None

    def convert_to_world_coordinates(self, depth_image, bounding_box):
        # # RGB camera parameters
        # cx = rgb_principal_point[0]
        # cy = rgb_principal_point[1]
        # fx = rgb_focal_length[0]
        # fy = rgb_focal_length[1]
        
        # IR camera parameters
        cx = infrared_principal_point[0]
        cy = infrared_principal_point[1]
        fx = infrared_focal_length[0]
        fy = infrared_focal_length[1]


        if bounding_box is not None:
            (x, y, w, h) = bounding_box

            # apply the quadratic calibration to the depth data
            camera_read = depth_image[int(y+h/2), int(x+w/2)]
            calibrated_depth = 0.0007415470919300713 * (camera_read**2) - 0.3236496146104614 * camera_read + 461.469635957998

            # convert to world coordinates
            u = int(x+w/2)
            v = int(y+h/2)
            world_X = (u - cx) * calibrated_depth / fx
            world_Y = (v - cy) * calibrated_depth / fy
            world_Z = calibrated_depth

            # create a PointStamped message
            point_camera_frame = PointStamped()
            point_camera_frame.header.stamp = self.get_clock().now().to_msg()
            point_camera_frame.header.frame_id = 'kinect2_ir_optical_frame'#'kinect2_rgb_optical_frame' 
            point_camera_frame.point.x = world_X
            point_camera_frame.point.y = world_Y
            point_camera_frame.point.z = world_Z

            # transform the point to the map frame
            try:
                transform = self.tf_buffer.lookup_transform('map',  # target frame
                                                            'kinect2_ir_optical_frame',  # source frame
                                                            rclpy.time.Time(),  # time
                                                            rclpy.duration.Duration(seconds=1.0))  # timeout
                point_map_frame = tf2_geometry_msgs.do_transform_point(point_camera_frame, transform)
                return (point_map_frame.point.x, point_map_frame.point.y, point_map_frame.point.z)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                self.get_logger().warn('Transform lookup failed: %s' % ex)
                return None


def main(args=None):
   rclpy.init(args=args)
   poop_detector_node = PoopDetectorNode()
   rclpy.spin(poop_detector_node)


   poop_detector_node.destroy_node()
   rclpy.shutdown()


if __name__ == "__main__":
   main()
   
