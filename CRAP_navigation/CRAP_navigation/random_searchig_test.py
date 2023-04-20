import math
import random
import time
import numpy as np
import rclpy
from geometry_msgs.msg import (OccupancyGrid, Point, PoseStamped, Quaternion,
                               Twist)
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class PoopMove(Node):

    def __init__(self):
        super().__init__('poop_move')
        

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


        # wait for transform from map to base_footprint
        self.get_logger().info('Waiting for transform from map to base_footprint')
        self.tf_buffer.wait_for_transform('base_footprint', 'odom', rclpy.time(), rclpy.duration.Duration(seconds=30))
        self.get_logger().info('Transform from odom to base_footprint available')
        
        # obtain initial pose from map to base_footprint transform
        try:
            # initial_transform = self.nav.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.now())
            # initial_pose.pose.position.x = initial_transform.transform.translation.x
            # initial_pose.pose.position.y = initial_transform.transform.translation.y
            # initial_pose.pose.position.z = initial_transform.transform.translation.z
            # initial_pose.pose.orientation = initial_transform.transform.rotation
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.01
            initial_pose.pose.position.y = 0.01
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
            self.get_logger.info('Initial pose: {}'.format(initial_pose))
        except Exception as e:
            self.get_logger.error('Failed to obtain initial pose: {}'.format(e))
        
        # set initial pose
        self.nav.set_initial_pose(initial_pose)
        self.get_logger().info('Initial pose set')

        # start navigation
        self.nav.waitUntilNav2Active()
        self.get_logger().info('PoopMove initialized, navigation lifecycle started')

        time.sleep(5)
        # Subscribe to /global_costmap/costmap of type OccupancyGrid

        # self.subscription = self.create_subscription(
        #     OccupancyGrid,
        #     '/global_costmap/costmap',
        #     self.Maplistener_callback,
        #     10)

        # set iteration flag
        self.plannerEnableFlag = True

        # set costmap threshold
        self.cost_threshold = 20


    def Maplistener_callback(self, msg):
        self.costmap_data = np.array(msg.data, dtype=np.int8)
        self.costmap_data = self.costmap_data.reshape(msg.info.height, msg.info.width)
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = [msg.info.origin.position.x, msg.info.origin.position.y]


    def timer_callback(self):

        self.costmap_data = np.array(self.nav.global_costmap.data, dtype=np.int8)
        self.costmap_data = self.costmap_data.reshape(self.nav.global_costmap.info.height, self.nav.global_costmap.info.width)
        self.costmap_resolution = self.nav.global_costmap.info.resolution
        self.costmap_origin = [self.nav.global_costmap.info.origin.position.x, self.nav.global_costmap.info.origin.position.y]

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
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
            goal_pose.pose.position.x = selected_point[0]
            goal_pose.pose.position.y = selected_point[1]

            self.nav.goToPose(goal_pose)

            self.plannerEnableFlag = False 
        else:
            self.get_logger().info('Planner disabled, navigating to search targets')
            # check if navigation is done
            if self.nav.isTaskComplete(trackingRoute=False):
                self.get_logger().info('Navigation done')
            else:
                self.get_logger().info('Navigation progressing')

    


def main(args=None):
    rclpy.init(args=args)
    poop_move = PoopMove()
    rclpy.spin(poop_move)
    poop_move.destroy_node()
    rclpy.shutdown()

