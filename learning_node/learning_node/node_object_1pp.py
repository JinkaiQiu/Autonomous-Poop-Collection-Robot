# node_object_1pp.py


# Depth coordinate without the shape filter only the color filter


#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2节点示例-通过摄像头识别检测图片中出现的苹果
"""
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import necessary modules
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类


import cv2                              # OpenCV图像处理库
import numpy as np                      # Python数值计算库


lower_brown = np.array([10, 50, 60])    # Poop的HSV阈值下限
upper_brown = np.array([40, 225, 255])  # Poop的HSV阈值上限


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
        except CvBridgeError as e:
            print(e)
            return
        if self.depth_image is not None:
            world_coords = self.object_detect(cv_image, self.depth_image)
            if world_coords is not None:
                print(f"World Coordinates: {world_coords}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            print(e)
            return
        
    def object_detect(self, image, depth_image):
        # define your camera parameters
        cx = 261.9911
        cy = 210.5512
        fx = 361.1934
        fy = 362.1103

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
            point_camera_frame.header.frame_id = 'kinect2_link'  # replace with your Kinect2 frame ID
            point_camera_frame.point.x = world_X
            point_camera_frame.point.y = world_Y
            point_camera_frame.point.z = world_Z

            # transform the point to the map frame
            try:
                transform = self.tf_buffer.lookup_transform('map',  # target frame
                                                            'kinect2_link',  # source frame
                                                            rclpy.time.Time(),  # time
                                                            rclpy.duration.Duration(seconds=1.0))  # timeout
                point_map_frame = tf2_geometry_msgs.do_transform_point(point_camera_frame, transform)
                return (point_map_frame.point.x, point_map_frame.point.y, point_map_frame.point.z)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                self.get_logger().warn('Transform lookup failed: %s' % ex)
                return None

        cv2.imshow("object", image)
        cv2.waitKey(500)
        return None



def main(args=None):
   rclpy.init(args=args)
   poop_detector_node = PoopDetectorNode()
   rclpy.spin(poop_detector_node)


   poop_detector_node.destroy_node()
   rclpy.shutdown()


if __name__ == "__main__":
   main()
