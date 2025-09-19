#i/user/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class turtlePublisher(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.angular.z = 0.5
        msg.linear.x = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    node = turtlePublisher()

    rclpy.spin(node)

    stop_msg=Twist()
    node.publisher_.publish(stop.msg)
    node.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
