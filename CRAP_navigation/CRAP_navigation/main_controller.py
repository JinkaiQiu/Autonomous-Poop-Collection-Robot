import math
import random
import time
import numpy as np
import rclpy
from geometry_msgs.msg import (Point, PoseStamped, Quaternion,PointStamped,
                               Twist)
from nav_msgs.msg import OccupancyGrid
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import IsPathValid
from math import sin, cos, pi
from rclpy.action import ActionClient
from serial_motor_demo_msgs.action import Collect

class crap_main_controller (Node):
	def __init__(self):
		super().__init__('crap_main_controller')
  	
		# Create Timer
		timer_period = 1.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		# Create detection timeout
		self.lastDetectTime = time.time() - 10000
		self.detectionTimeout = 10
		
		# initialize NAV2 Basic Navigator
		self.nav = BasicNavigator()
		self.get_logger().info('PoopMove initialized, navigation lifecycle started')
		time.sleep(1)
		
		# enable TF_listener
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.get_logger().info('Tf buffer and listener initialized')
		
		# Create Flags
		self.poopExists = False
		self.cameraInputExists = False
		
	def timer_callback(self):
		# Camera is detecting poop location or not 
		self.cameraInputExists = (time.time() - self.lastDetectTime) < self.detectionTimeout
		
		# if detection time < timeout, poop exists, run navigation
		if self.cameraInputExists:
			self.get_logger().info('Case 1: detected and navigating to initial capturing pose ...')
			self.poopExists = True
		
		elif self.poopExists and not self.CameraInputExists: 
			self.get_logger().info('Case 2: poop goes blind and actuate capturing sequence ...')
			
			# if grabbing action complte, set poopExists to False, run backup 
		
		else:
			self.get_logger().info('Case 3: no poop exists, searching ...')
