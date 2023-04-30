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


class PoopMove(Node):

    def __init__(self):
        super().__init__('poop_move')
        
        # publish goal pose 
        self.goalPose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)


        # create timer
        timer_period = 1.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lastDetectTime = time.time() - 10000;
        self.detectionTimeout = 2

        self.navOffset = 0.2 # offset from ball to stop
        self.navStopThreshold = 0.5 # threshold to stop navigation
        self.ballLocMap = [0,0] # ball location in map frame

        # initialize 
        self.nav = BasicNavigator()
        self.get_logger().info('PoopMove initialized, navigation lifecycle started')
        time.sleep(1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('Tf buffer and listener initialized')
        
        self.costmap_data = None
        self.costmap_origin = None
        self.costmap_resolution = None
        
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

        time.sleep(5)
        
        # Subscribe to /global_costmap/costmap of type OccupancyGrid
        self.subscription = self.create_subscription(
             OccupancyGrid,
             '/global_costmap/costmap',
             self.Maplistener_callback,
             10)
        self.get_logger().info('Cost map subscriber created')

        # set iteration flag
        self.plannerEnableFlag = True

        # set costmap threshold
        self.cost_threshold = 20


    def timer_callback(self):
        if self.plannerEnableFlag:
            # If plannerFlag is true, enable planner
            if self.costmap_data is None:
                self.get_logger().warning('Costmap data is None')
                return
            self.get_logger().info('Planner enabled')
            # plan path to search target
            low_cost_indices = np.argwhere(self.costmap_data < self.cost_threshold)
            if len(low_cost_indices) < 1:
                self.get_logger().error('No low-cost points found.')
            
            reachable = False
            # for the low_cost point, use nav2 to check if it is reachable
            while not reachable:
                selected_index = random.choice(low_cost_indices)
                selected_point = np.array([self.costmap_resolution* selected_index[1] + self.costmap_origin[0],
                                        self.costmap_resolution* selected_index[0] + self.costmap_origin[1]])
                
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

                # publish goal pose
                self.goalPose_pub.publish(goal_pose)

                status = self.nav.goToPose(goal_pose)

                # if path is None:
                if status == False:
                    self.get_logger().info('Path not found, retrying')
                else:
                    reachable = True
                    self.get_logger().info('Path found, navigating to search target')
                    self.plannerEnableFlag = False 

        else:
            self.get_logger().info('Planner disabled, navigating to search targets')
            # check if navigation is done

            if self.nav.isTaskComplete():
                self.get_logger().info('Navigation done')
                self.plannerEnableFlag = True
                reachable = False
            else:
                self.get_logger().info('Navigation progressing')
        
    
    def Maplistener_callback(self, msg):
        self.costmap_data = np.array(msg.data, dtype=np.int8)
        self.costmap_data = self.costmap_data.reshape(msg.info.height, msg.info.width)
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        # self.get_logger().info('Map received')


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

