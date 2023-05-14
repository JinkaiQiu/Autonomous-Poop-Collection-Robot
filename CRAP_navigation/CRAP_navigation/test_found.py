import math
import random
import time
import numpy as np
import rclpy
from geometry_msgs.msg import (Point, PoseStamped, Quaternion,PointStamped,
                               Twist)
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import IsPathValid
from math import sin, cos, pi
from rclpy.action import ActionClient
from serial_motor_demo_msgs.action import Collect

class PoopMove(Node):

    def __init__(self):
        super().__init__('poop_move')
        
        # publish goal pose 
        self.goalPose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)


        # create timer
        timer_period = 1.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lastDetectTime = time.time() - 10000;
        self.detectionTimeout = 3

        self.navOffset = 0.01 # offset from ball to stop
        self.navStopThreshold = 0.2 # threshold to stop navigation
        self.ballLocMap = [0,0] # ball location in map frame

        # initialize 
        self.nav = BasicNavigator()
        self.get_logger().info('PoopMove initialized, navigation lifecycle started')
        time.sleep(1)

        self.poopCollect = ActionClient(self, Collect, 'collect_poop')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('Tf buffer and listener initialized')
        
        # obtain initial pose from map to base_footprint transform
        try:
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.01
            initial_pose.pose.position.y = 0.01
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
            self.get_logger().info('Initial pose: {}'.format(initial_pose))
        except Exception as e:
            self.get_logger().error('Failed to obtain initial pose: {}'.format(e))
        
        # set initial pose
        self.nav.setInitialPose(initial_pose)
        self.get_logger().info('Initial pose set')

        # start navigation
        self.nav.waitUntilNav2Active()
        self.get_logger().info('PoopMove initialized, navigation lifecycle started')

        time.sleep(3)

        # subscribe to /ball_location of type PointStamped
        self.subscription = self.create_subscription(
            PointStamped,
            '/ball_location',
            self.BallLoclistener_callback,
            10)
        self.get_logger().info('Ball location subscriber created')
        
        self.notArrived = True

        self.firstTimePlan = True
        # self.firstTimeSucc = True
        self.firstTimeAction = True
        self.collected = False

    def timer_callback(self):
        # if detection time smaller than timeout, navigate to ball
        if (time.time() - self.lastDetectTime) < self.detectionTimeout: 
            self.get_logger().info('Detected')
            if self.notArrived:
                # cancel current navigation task
                if self.firstTimePlan:
                    self.nav.cancelTask()
                    self.firstTimePlan = False
                # get transform from map to base_footprint
                baseLocMap = self.findRobot()
                
                # calculate vector from base_footprint to ball
                ballVecMap = np.array([self.ballLocMap[0] - baseLocMap[0], self.ballLocMap[1] - baseLocMap[1]])
                ballVecMapUnit = ballVecMap/np.linalg.norm(ballVecMap)    
                ballVecLen = np.linalg.norm(ballVecMap)
                
                self.get_logger().info('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX')
                self.get_logger().info('Distance to ball: {}'.format(ballVecLen))
                self.get_logger().info('Base to Ball vector: {}'.format(ballVecMap))
                self.get_logger().info('Base to Ball unit vector: {}'.format(ballVecMapUnit))
                self.get_logger().info('Base location: {}'.format(baseLocMap))
                self.get_logger().info('Ball location: {}'.format(self.ballLocMap))
                self.get_logger().info('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX')

                # if ball is not close enough, navigate to ball
                if ballVecLen > self.navStopThreshold:
                    self.get_logger().info('Task Refresshing')
                    # calculate yaw angle
                    yaw = math.atan2(ballVecMapUnit[1], ballVecMapUnit[0])
                    
                    # convert yaw angle to quaternion, set goal orientation
                    goal_orientation = Quaternion()
                    goal_orientation.x = 0.0
                    goal_orientation.y = 0.0
                    goal_orientation.z = math.sin(yaw/2)
                    goal_orientation.w = math.cos(yaw/2)

                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
                    goal_pose.pose.position.x = self.ballLocMap[0] - ballVecMapUnit[0]*self.navOffset
                    goal_pose.pose.position.y = self.ballLocMap[1] - ballVecMapUnit[1]*self.navOffset
                    goal_pose.pose.orientation = goal_orientation
                    # send goal
                    self.nav.goToPose(goal_pose)
            
                else:
                    self.get_logger().info('Task Succeed')
                    # if self.firstTimeSucc:
                    self.nav.cancelTask()
                     #   self.firstTimeSucc = False
                    self.notArrived = False
            else:
                self.get_logger().info('Arrived, grabbing enabled')
                # Run grabbing action
                if self.firstTimeAction:
                    self.send_goal()
                    self.firstTimeAction = False
                if self.collected:
                    self.get_logger().info('Grabbing complete')
                    self.notArrived = True
                    self.firstTimePlan = True
                    self.firstTimeAction = True
                else:
                    self.get_logger().info('Grabbing in progress')

        else: # if detection time larger than timeout, stop navigation, search for ball
            self.get_logger().info('Target lost')
            # self.nav.spin()
            self.notArrived = True
            self.firstTimePlan = True
            # self.firstTimeSucc = True

    
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

    def BallLoclistener_callback(self, msg):
        f = 0.5 # filter coefficient
        # self.get_logger().info('Received: {} {}'.format(msg.point.x, msg.point.y))
        self.lastDetectTime = time.time()
        # low pass filter for ball location to reduce noise
        self.ballLocMap = [f*self.ballLocMap[0] + (1-f)*msg.point.x, f*self.ballLocMap[1] + (1-f)*msg.point.y]

    def findRobot (self):
        # find robot location in map frame
        baseTransform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=1))
        baseLocMap = [baseTransform.transform.translation.x, baseTransform.transform.translation.y]
        return baseLocMap
    
    def get_robot_pose(self):
        try:
            # Get the transform from the map frame to the robot's frame
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Create a PoseStamped message
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose
        except Exception as e:
            self.get_logger().error('Failed to get robot pose: {}'.format(e))
            return None


def main(args=None):
    rclpy.init(args=args)
    poop_move = PoopMove()
    rclpy.spin(poop_move)
    # poop_move.destroy_node()
    # rclpy.shutdown()

