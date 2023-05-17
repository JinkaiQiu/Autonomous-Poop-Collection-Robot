import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import random

class BallPublisher(Node):
    def __init__(self):
        super().__init__("ball_publisher")
        self.publisher_ = self.create_publisher(PointStamped, "/ball_location", 10)
        self.timer_ = self.create_timer(0.75, self.publish_ball_location)
        
    def publish_ball_location(self):
        ball_msg = PointStamped()
        ball_msg.header.frame_id = "map"
        ball_msg.header.stamp = self.get_clock().now().to_msg()
        ball_msg.point.x = 3.5 + random.uniform(-0.1, 0.1) # Add randomized noise to x coordinate
        ball_msg.point.y = -4.5 + random.uniform(-0.1, 0.1) # Add randomized noise to y coordinate
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