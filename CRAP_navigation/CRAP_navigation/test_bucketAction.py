import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from serial_motor_demo_msgs.action import Collect
import action_msgs  

class PoopMove(Node):
    def __init__(self):
        super().__init__('test_bucketAction')
        self.poopCollect = ActionClient(self, Collect, 'collect_poop')
        timer_period = 1.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.firstTimeAction = True
        self.collected = False

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

    
    def timer_callback(self):
        if self.firstTimeAction:
            self.send_goal()
            self.firstTimeAction = False

        if self.collected:
            self.get_logger().info('Grabbing complete')
        else:
            self.get_logger().info('Grabbing in progress')


def main(args=None):
    rclpy.init(args=args)
    test_bucketAction = PoopMove()
    rclpy.spin(test_bucketAction)