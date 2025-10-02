import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time

heartbeat_period = 0.1
time_window = 20.0
threshold = 238



class Lab_3Assignment_5(Node):

    def __init__(self):
        super().__init__('lab_3_assignment_5')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.feature_sub = self.create_subscription(Float64, '/feature_mean', self.feature_callback, 10)

        self.count_open = 0
        self.count_closed = 0
        self.total = 0

        self.start_time = time.time()
        self.log.info("Assignment 5 node started, collecting data...")
    
    def feature_callback(self, msg):
        self.feature_val = msg.data


    def heartbeat(self):
        now = time.time()
        elapsed = now - self.start_time

        if self.feature_val is not None:
            if self.feature_val <= threshold:
                self.count_open += 1
                state = "OPEN"
            else:
                self.count_closed += 1
                state = "CLOSED"

            self.total += 1
            # Debug print for each measurement
            self.log.info(f"feature_mean={self.feature_val:.2f} → {state}")

        if elapsed >= time_window:
            self.report_results()
            rclpy.shutdown()

    def report_results(self):
        self.log.info("---- Feature-only Results ----")
        self.log.info(f"Total samples: {self.total}")
        self.log.info(f"Count (>= threshold, open):   {self.count_open}")
        self.log.info(f"Count (< threshold, closed): {self.count_closed}")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    lab_3_assignment_5 = Lab_3Assignment_5()
    lab_3_assignment_5.spin()
    lab_3_assignment_5.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
