import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


heartbeat_period = 0.1

class lab_2_question_4(Node):

    def __init__(self):
        super().__init__('lab_2_question_4')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.door_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.torque_value = 2.0

        self.declare_parameter('forward_speed', 0.5)   # default = 0.5 m/s
        self.vel = self.get_parameter('forward_speed').get_parameter_value().double_value

        self.state = 0
        self.counter = 0
        
        self.log.info(f'Node started with forward_speed={self.vel:.3f} m/s')

    def heartbeat(self):
        if self.state == 0:
            self.publish_torque(self.torque_value)
            self.counter += 1
            if self.counter > 40:
                self.counter = 0
                self.state += 1
        if self.state == 1:
            self.publish_vel(self.vel)
            self.counter += 1
            if self.counter > 100:
                self.counter = 0
                self.state += 1
        if self.state == 2:
            self.publish_vel(0)
            self.publish_torque(-self.torque_value)
            self.counter += 1
            if self.counter > 20:
                self.counter = 0
                self.state += 1


    def publish_torque(self, torque):
        msg = Float64()
        msg.data = torque
        self.door_pub.publish(msg)
        self.get_logger().info(f'Publishing torque: {msg.data:.2f} Nm')

    def publish_vel(self, vel):
        msg = Twist()
        msg.linear.x = float(vel)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Publishing velocity: {msg.linear.x:.2f} m/s')


def main():
    rclpy.init()
    node = lab_2_question_4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
