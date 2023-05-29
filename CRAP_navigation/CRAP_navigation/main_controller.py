import math
import random
import time
import numpy as np
import rclpy
from geometry_msgs.msg import (Point, PoseStamped, Quaternion,PointStamped,
                               Twist)
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from CRAP_navigation.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import IsPathValid
from math import sin, cos, pi
from rclpy.action import ActionClient
from serial_motor_demo_msgs.action import Collect
from threading import Lock

class crap_main_controller (Node):
	def __init__(self):
		super().__init__('crap_main_controller')
  	
		# Create Timer
		timer_period = 1.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		# initialize NAV2 Basic Navigator
		self.nav = BasicNavigator()
		time.sleep(3)
		self.initial_pose_set()
		self.nav.waitUntilNav2Active()
		self.get_logger().info('Navigation Lifecycle Started')
		
		# initialize arduino movement client
		self.poopCollect = ActionClient(self, Collect, 'collect_poop')

		# initialize TF_listener
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.get_logger().info('Tf buffer and listener initialized')

		# subscribe to /ball_location of type PointStamped
		self.subscriptionBallLoc = self.create_subscription(
			PointStamped,
			'/poop_coordinates',
			self.BallLoclistener_callback,
			10)
		self.get_logger().info('Ball location subscriber created')

		# Subscribe to /global_costmap/costmap of type OccupancyGrid
		self.subscriptionCostMap = self.create_subscription(
				OccupancyGrid,
				'/global_costmap/costmap',
				self.Maplistener_callback,
				10)
		self.get_logger().info('Cost map subscriber created')

		# create detection variables
		self.lastDetectTime = time.time() - 10000
		self.detectionTimeout = 10

		# create navigation flags
		self.navigationInProgress = False

		self.detectedTimes = 0
		self.refreshingNeeded = True
		self.forwarded = False

		# create costmap variables
		self.costmap_data = None
		self.costmap_origin = None
		self.costmap_resolution = None

		# create Flags
		self.cameraInputExists = False
		self.readyToCapture = False
		self.grabbingInProgress = False 
		self.collected = False

		# create thread lock
		self.callback_lock = Lock()

		
	################################################################################################################
	# main logic control function
	def timer_callback(self):
		# if not self.callback_lock.acquire(blocking=False):
		# 	self.get_logger().warn('Callback lock is held, skipping callback')
		# 	return

		if self.costmap_data is None:
			self.get_logger().warn('No costmap data, skipping callback')
			return

		try:
			# Camera is detecting poop location or not 
			self.cameraInputExists = (time.time() - self.lastDetectTime) < self.detectionTimeout
			
			# if poop exists in camera view, but robot not in position
			if self.cameraInputExists and not self.readyToCapture:
				self.get_logger().info('Case 1: detected and navigating to initial capturing pose ...')
				
				# refresh iteration count, only run once
				if self.navigationInProgress:
					self.navigationInProgress = False
					self.get_logger().info('iterator started ...')
					self.detectedTimes = 0
					self.refreshingNeeded = True
					self.forwarded = False
				
				if self.detectedTimes%5 == 0 and self.refreshingNeeded: # run every 5 times of detection
					goal_pose = self.getNavTargetToPoop()
					self.refreshNavigationGoal(goal_pose)

				# check if navigation is complete
				if not self.forwarded:
					navReached = self.getNavigationFeedbackAndCheck()
					if not navReached:
						self.get_logger().info('Navigating to initial capturing pose ...')
						self.detectedTimes += 1
						return
					else:
						self.get_logger().info('Initial capturing pose reached!')
						self.refreshingNeeded = False # no need to refresh goal pose anymore 
						
						# if need to tune distance, add here
						# self.nav.backup(backup_dist=-0.2,backup_speed=0.2)
						
						goal_pose = self.getNavTargetToPoop(target_offset=-0.5)
						self.refreshNavigationGoal(goal_pose)

						self.forwarded = True
				else: 
					navReached = self.getNavigationFeedbackAndCheck()
					if not navReached:
						self.get_logger().info('Driving to poop ...')
						return
					else:
						self.get_logger().info('Poop reached!')
						self.readyToCapture = True	

			# robot has moved to position, ready to capture
			elif self.readyToCapture: 
				self.get_logger().info('Case 2: in position, actuate capturing ...')
				# run poop capturing sequence
				if not self.grabbingInProgress:
					self.send_goal()
					self.grabbingInProgress = True
					return
				else: 
					self.get_logger().info('Collection in progress ...')
				
				if self.collected:
					self.get_logger().info('Poop collected')
					if not self.navigationInProgress:
						self.nav.backup(backup_dist=0.3,backup_speed=0.2,time_allowance=10)
						self.navigationInProgress = True
						self.get_logger().info('Start to backup to check')
						return
					else: 
						backupComplete = self.getNavigationFeedbackAndCheck()
						if backupComplete:
							#refresh flags
							self.readyToCapture = False
							self.grabbingInProgress = False
							self.collected = False
							self.detectedTimes = 0
							#added
							self.navigationInProgress = False
						else:
							self.get_logger().info('Backing up ...')


			# no robot present, start to search	
			else:
				self.get_logger().info('Case 3: no poop exists, searching ...')
				if not self.navigationInProgress:
					self.nav.cancelTask()
					goalPose = self.goToRandomPointInCostmap()
					if goalPose is None :
						self.get_logger().error('No low-cost points found.')
						return
					self.navigationInProgress = True
					return
				else:
					self.getNavigationFeedbackAndCheck()
		finally:
			# self.callback_lock.release()
			self.get_logger().info('__________________________________')
	################################################################################################################
	
	def initial_pose_set(self,x= 0.01,y = 0.01,z = 0.0,w = 0.1):
		try:
			initial_pose = PoseStamped()
			initial_pose.header.frame_id = 'map'
			initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
			initial_pose.pose.position.x = x
			initial_pose.pose.position.y = y
			initial_pose.pose.orientation.z = z
			initial_pose.pose.orientation.w = w
			self.get_logger().info('Initial pose: {}'.format(initial_pose))
		except Exception as e:
			self.get_logger().error('Failed to obtain initial pose: {}'.format(e))

		# set initial pose
		self.nav.setInitialPose(initial_pose)
		self.get_logger().info('Initial pose set')

	# callback functions for arduino movements 
	def send_goal(self):
		self.poopCollect.wait_for_server()
		goal_msg = Collect.Goal()
		goal_msg.command = 'enable'
		self.send_goal_future = self.poopCollect.send_goal_async(goal_msg)
		self.send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return
		self.get_logger().info('Goal accepted')
		self.get_result_future = goal_handle.get_result_async()
		self.get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.collected = result.success
	
	# callback function for ball_location subscriber
	def BallLoclistener_callback(self, msg):
		# assume already filtered by camera node
		self.lastDetectTime = time.time()
		self.ballLocMap = [msg.point.x, msg.point.y]

	# callback function for costmap subscriber
	def Maplistener_callback(self, msg):
			self.costmap_data = np.array(msg.data, dtype=np.int8)
			self.costmap_data = self.costmap_data.reshape(msg.info.height, msg.info.width)
			self.costmap_resolution = msg.info.resolution
			self.costmap_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
			# self.get_logger().info('Map received')

	# generate random points in costmap
	def goToRandomPointInCostmap(self,threshold = 10,distance_threshold = 1):
		low_cost_indices = np.argwhere(self.costmap_data < threshold)
		if len(low_cost_indices) < 1:
			self.get_logger().error('No low-cost points found.')
			return None
		reachable = False
		while not reachable:
			selected_index = random.choice(low_cost_indices)
			selected_point = np.array([self.costmap_resolution* selected_index[1] + self.costmap_origin[0],
									self.costmap_resolution* selected_index[0] + self.costmap_origin[1]])		
			
			baseLocMap = self.findRobot()
			distance = np.sqrt((baseLocMap[0] - selected_point[0])**2 + (baseLocMap[1] - selected_point[1])**2)
			if distance < distance_threshold:
				self.get_logger().info('Selected point too close to robot, retrying')
				continue

			goal_pose = PoseStamped()
			goal_pose.header.frame_id = 'map'
			goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
			goal_pose.pose.position.x = float(selected_point[0])
			goal_pose.pose.position.y = float(selected_point[1])        
			theta = 2 * pi * random.random()
			q = Quaternion()
			q.w = cos(theta / 2)
			q.x = 0.0
			q.y = 0.0
			q.z = sin(theta / 2)
			goal_pose.pose.orientation = q

			status = self.nav.goToPose(goal_pose)

			# if path is None:
			if status == False:
				self.get_logger().info('Path not found, retrying')
			else:
				reachable = True
				self.get_logger().info('Path found, navigating to search target')
				self.navigationInProgress = True
				return goal_pose
			
	def interpretNavGoalResult(self):
		result = self.nav.getResult()
		if result == TaskResult.SUCCEEDED:
			self.get_logger().info('Goal succeeded!')
			return True
		elif result == TaskResult.CANCELED:
			self.get_logger().warn('Goal was canceled!')
			return False
		elif result == TaskResult.FAILED:
			self.get_logger().error('Goal failed!')
			return False
		else:
			self.get_logger().warn('Goal has an invalid return status!')
			return False

	def getNavigationFeedbackAndCheck(self):
		self.nav.getFeedback()
		if self.nav.isTaskComplete():
			self.navigationInProgress = False
			self.get_logger().info('Navigation complete')
			result = self.interpretNavGoalResult()
			return result
		else:
			self.get_logger().info('Navigating ...')
			# find robot position and log
			baseLocMap = self.findRobot()
			self.get_logger().info('Robot location: {}'.format(baseLocMap))
			return False
		
	def refreshNavigationGoal(self, goal_pose):
		self.nav.cancelTask()
		self.nav.goToPose(goal_pose)
		self.get_logger().info('New navigation goal set')
   
	def findRobot (self):
		# find robot location in map frame
		baseTransform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=1))
		baseLocMap = [baseTransform.transform.translation.x, baseTransform.transform.translation.y]
		return baseLocMap

	def getNavTargetToPoop(self, target_offset=1):
		# position 
		baseLocMap = self.findRobot()
		ballVecMap = np.array([self.ballLocMap[0] - baseLocMap[0], self.ballLocMap[1] - baseLocMap[1]])
		ballVecLen = np.linalg.norm(ballVecMap)
		ballVecMapUnit = ballVecMap/ballVecLen  
		navTargetVecMap = ballVecMapUnit * (ballVecLen - target_offset)

		# orientation
		yaw = math.atan2(ballVecMapUnit[1], ballVecMapUnit[0])        
		goal_orientation = Quaternion()
		goal_orientation.x = 0.0
		goal_orientation.y = 0.0
		goal_orientation.z = math.sin(yaw/2)
		goal_orientation.w = math.cos(yaw/2)
		
		# formulate goal pose
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
		goal_pose.pose.orientation = goal_orientation
		goal_pose.pose.position.x = baseLocMap[0] + navTargetVecMap[0]
		goal_pose.pose.position.y = baseLocMap[1] + navTargetVecMap[1]

		return goal_pose
	
def main(args=None):
    rclpy.init(args=args)
    Crap_Controller = crap_main_controller()
    rclpy.spin(Crap_Controller)
    # poop_move.destroy_node()
    # rclpy.shutdown()
