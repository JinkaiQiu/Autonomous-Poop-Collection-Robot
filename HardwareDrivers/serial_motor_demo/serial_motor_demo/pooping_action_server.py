import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from serial_motor_demo_msgs.action import Collect
import serial
import time

class PoopCollectionActionServer(Node):

    def __init__(self):
        super().__init__('poop_collection_action_server')
        self._action_server = ActionServer(
            self,
            Collect,
            'collect_poop',
            self.execute_callback)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600) #may pending on the port
            if not self.ser.isOpen():
                raise IOError("Serial port could not be opened.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            raise e

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        msg = String()
        msg.data = "enable"
        self.ser.write(msg.data.encode())
        result = Collect.Result()
        result.success = False
        # start_time = time.time()
        # timeout = 60  # Timeout after 30 seconds
        while result.success == False:
            # if time.time() - start_time > timeout:
            #     result.success = False
            #     self.get_logger().info('Action timed out.')
            #     break

            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').rstrip()
                self.get_logger().info(f'Arduino Sent: {line}')
                if line == "complete":
                    result.success = True
                    self.get_logger().info('Arduino Action Completed')
                    break


        goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)

    poop_collection_action_server = PoopCollectionActionServer()

    rclpy.spin(poop_collection_action_server)


if __name__ == '__main__':
    main()
