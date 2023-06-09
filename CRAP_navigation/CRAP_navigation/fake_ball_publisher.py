import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped
import random

class BallPublisher(Node):
    def __init__(self):
        super().__init__("ball_publisher")
        self.publisher_ = self.create_publisher(PointStamped, "/poop_coordinates", 10)
        self.timer_ = self.create_timer(1.5, self.publish_ball_location)
        self.declare_parameter('x', 2.0)
        self.declare_parameter('y', 0.0)
        # ros2 run CRAP_navigation fake_ball_publisher --ros-args -p x:=1.0 -p y:=-2.0

    def publish_ball_location(self):
        ball_msg = PointStamped()
        ball_msg.header.frame_id = "map"
        ball_msg.header.stamp = self.get_clock().now().to_msg()
        ball_msg.point.x = self.get_parameter('x').value + random.uniform(-0.05, 0.05) # Add randomized noise to x coordinate
        ball_msg.point.y = self.get_parameter('y').value + random.uniform(-0.05, 0.05) # Add randomized noise to y coordinate
        ball_msg.point.z = 0.0
        self.publisher_.publish(ball_msg)

def main(args=None):
    rclpy.init(args=args)
    ball_publisher = BallPublisher()
    rclpy.spin(ball_publisher)
    ball_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()