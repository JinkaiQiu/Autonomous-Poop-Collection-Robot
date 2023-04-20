import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Twist, PoseStamped, Quaternion, OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
from tf2_ros import TransformListener, Buffer
import numpy as np
import math
import random

class PoopMove(Node):

    def __init__(self):
        super().__init__('poop_move')

        # subscribe to /detected_ball of type Point
        self.subscription = self.create_subscription(
            Point,
            'detected_ball',
            self.listener_callback,
            10)
        
        # Subscribe to /global_costmap/costmap of type OccupancyGrid
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.Maplistener_callback,
            10)

        # create timer
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lastDetectTime = time.time() - 10000;
        self.detectionTimeout = 2

        self.navOffset = 0.2 # offset from ball to stop
        self.navStopThreshold = 0.5 # threshold to stop navigation
        self.ballLocMap = [0,0] # ball location in map frame

        # initialize navigation
        self.nav = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()

        # wait for transform from map to base_footprint
        self.get_logger().info('Waiting for transform from map to base_footprint')
        self.tf_buffer.wait_for_transform('base_footprint', 'map', rclpy.time(), rclpy.duration.Duration(seconds=30))
        self.get_logger().info('Transform from map to base_footprint available')
        
        # obtain initial pose from map to base_footprint transform
        try:
            initial_transform = self.nav.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.now())
            initial_pose.pose.position.x = initial_transform.transform.translation.x
            initial_pose.pose.position.y = initial_transform.transform.translation.y
            initial_pose.pose.position.z = initial_transform.transform.translation.z
            initial_pose.pose.orientation = initial_transform.transform.rotation
            self.get_logger.info('Initial pose: {}'.format(initial_pose))
        except Exception as e:
            self.get_logger.error('Failed to obtain initial pose: {}'.format(e))
        
        # set initial pose
        self.nav.set_initial_pose(initial_pose)
        self.get_logger().info('Initial pose set')

        # start navigation
        self.nav.lifecycleStartup()
        self.get_logger().info('PoopMove initialized, navigation lifecycle started')

        # set iteration flag
        self.plannerEnableFlag = True

        # set costmap threshold
        self.cost_threshold = 20

    def listener_callback(self, msg):
        f = 0.9 # filter coefficient
        self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))
        self.lastDetectTime = time.time()
        # low pass filter for ball location to reduce noise
        self.ballLocMap = [f*self.ballLocMap[0] + (1-f)*msg.x, f*self.ballLocMap[1] + (1-f)*msg.y]


    def Maplistener_callback(self, msg):
        self.costmap_data = np.array(msg.data, dtype=np.int8)
        self.costmap_data = self.costmap_data.reshape(msg.info.height, msg.info.width)
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = [msg.info.origin.position.x, msg.info.origin.position.y]


    def timer_callback(self):        
        # if detection time smaller than timeout, navigate to ball
        if (time.time() - self.lastDetectTime) < self.detectionTimeout: 
            self.get_logger().info('Detected')
            # cancel current navigation task
            self.nav.cancelTask()
            self.plannerEnableFlag = True 
            # get transform from map to base_footprint
            baseTransform = self.nav.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.now())
            baseLocMap = [baseTransform.transform.translation.x, baseTransform.transform.translation.y]
            
            # calculate vector from base_footprint to ball
            ballVecMap = np.array([self.ballLocMap[0] - baseLocMap[0], self.ballLocMap[1] - baseLocMap[1]])
            ballVecMapUnit = ballVecMap/np.linalg.norm(ballVecMap)    
            ballVecLen = np.linalg.norm(ballVecMap)
            
            # if ball is not close enough, navigate to ball
            if ballVecLen > self.navStopThreshold:
                self.get_logger().info('Distance to ball: {}'.format(ballVecLen))
                self.get_logger().info('Base to Ball vector: {}'.format(ballVecMap))

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

                self.get_logger().info('Goal pose: {}'.format(goal_pose))

                # send goal
                self.nav.goToPose(goal_pose)
            
            else:
                self.nav.cancelTask()
                self.get_logger().info('Stopped')          
            
        else: # if detection time larger than timeout, stop navigation, search for ball
            self.get_logger().info('Target lost')
            # cancel navigation if target is lost
            self.nav.cancelTask()
            # If plannerFlag is true, enable planner
            if self.plannerEnableFlag:
                self.get_logger().info('Planner enabled')
                # plan path to search target
                low_cost_indices = np.argwhere(self.costmap_data < self.cost_threshold)
                if len(low_cost_indices) < 1:
                    self.get_logger().error('No low-cost points found.')
                selected_index = random.choice(low_cost_indices)
                selected_point = np.array([[self.costmap_resolution* selected_index[1] + self.costmap_origin[0],
                                            self.costmap_resolution* selected_index[0] + self.costmap_origin[1]]]).T
                self.plannerEnableFlag = False 
            else:
                self.get_logger().info('Planner disabled')


    


def main(args=None):
    rclpy.init(args=args)
    poop_move = PoopMove()
    rclpy.spin(poop_move)
    poop_move.destroy_node()
    rclpy.shutdown()

