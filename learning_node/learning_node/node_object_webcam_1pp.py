# here is the Code with SHAPE & Color filter

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类

import cv2                              # OpenCV图像处理库
import numpy as np                      # Python数值计算库

lower_brown = np.array([10, 50, 60])    # Poop的HSV阈值下限
upper_brown = np.array([40, 225, 255])  # Poop的HSV阈值上限

# lower_brown = np.array([5, 30, 40])    # Poop的HSV阈值下限
# upper_brown = np.array([50, 200, 230])  # Poop的HSV阈值上限

class PoopDetectorNode(Node):
    def __init__(self):
        super().__init__("poop_detector_node")
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image,"/kinect2/qhd/image_color_rect", self.image_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, "/kinect2/qhd/image_depth_rect", self.depth_callback, 10)
        self.depth_image = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        if self.depth_image is not None:
            object_detect(cv_image, self.depth_image)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

# For approxiate Poop Shape
# Load the sample image of the poop
img = cv2.imread('src/learning_node/learning_node/sample3.png')

# Convert the image to grayscale and apply edge detection
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 100, 200)

# Find the contour of the poop shape
contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contour = max(contours, key=cv2.contourArea)

# Store the poop shape as a numpy array
poop_shape = np.array(contour)

def object_detect(image, depth_image):
    max_area = 0
    max_cnt = None
    second_max_area = 0
    second_max_cnt = None
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv_img, lower_brown, upper_brown)

    contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        if cnt.shape[0] < 150:
            continue

        area = cv2.contourArea(cnt)
        shape_match = cv2.matchShapes(poop_shape, cnt, 1, 0.0)
        #print(shape_match)

        if area > max_area and shape_match < 0.3:
            # the contour roughly fits the poop shape
            second_max_area = max_area
            second_max_cnt = max_cnt
            max_area = area
            max_cnt = cnt
        elif area > second_max_area and shape_match < 0.3:
            # the contour roughly fits the poop shape, but not as well as the largest contour
            second_max_area = area
            second_max_cnt = cnt

    if max_cnt is not None:
        (x, y, w, h) = cv2.boundingRect(max_cnt)
        # print("draw the big match!")  
        cv2.drawContours(image, [max_cnt], -1, (0, 255, 0), 2)
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)
        depth = depth_image[int(y+h/2), int(x+w/2)]
        print(f"Depth: {depth}")
    elif second_max_cnt is not None:
        (x, y, w, h) = cv2.boundingRect(second_max_cnt)
        # print("draw the second match")
        cv2.drawContours(image, [second_max_cnt], -1, (0, 255, 0), 2)
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)
        depth = depth_image[int(y+h/2), int(x+w/2)]
        print(f"Depth: {depth}")

    cv2.imshow("object", image)  # 使用OpenCV显示处理后的图像效果
    cv2.waitKey(500)

def main(args=None):
    rclpy.init(args=args)
    poop_detector_node = PoopDetectorNode()
    rclpy.spin(poop_detector_node)

    poop_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
