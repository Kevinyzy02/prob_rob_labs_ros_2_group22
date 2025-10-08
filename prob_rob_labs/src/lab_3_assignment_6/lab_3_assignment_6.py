import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import time

heartbeat_period = 0.1
feature_threshold = 238
bayes_threshold = 0.9999

P_z_o_x_o = 0.977
P_z_c_x_o = 0.023
P_z_c_x_c = 0.948
P_z_o_x_c = 0.052


class Lab_3Assignment_6(Node):

    def __init__(self):
        super().__init__('lab_3_assignment_5')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.feature_sub = self.create_subscription(Float64, '/feature_mean', self.feature_callback, 10)

        self.feature_val = None
        self.beliefs = [0.5]
        self.belief_open = 0.5


        self.start_time = time.time()
        self.log.info("Assignment 6 node started, starting Bayesian inference...")
    
    def feature_callback(self, msg):
        self.feature_val = msg.data


    def heartbeat(self):
        now = time.time()
        elapsed = now - self.start_time

        if self.feature_val is not None:
            if self.feature_val <= feature_threshold:
                z = "open"
                P_z_given_open = P_z_o_x_o
                P_z_given_closed = P_z_o_x_c
            else:
                z = "closed"
                P_z_given_open = P_z_c_x_o
                P_z_given_closed = P_z_c_x_c
            
            numerator = P_z_given_open * self.belief_open
            denominator = numerator + P_z_given_closed * (1.0-self.belief_open)
            posterior = numerator / denominator if denominator > 0 else self.belief_open

            self.belief_open = posterior
            self.beliefs.append(posterior)

            
            if posterior > bayes_threshold:
                decision = "OPEN" 
            elif posterior < (1-bayes_threshold):
                decision = "CLOSED"
            else:
                decision = "UNSURE"

            # Debug print for each measurement
            self.log.info(f"feature_mean={self.feature_val:.2f}, z = {z}, belief_open={posterior:.5f} → decision{decision}")

        if posterior > bayes_threshold or posterior < (1-bayes_threshold):
            elapsed = now - self.start_time
            self.log.info(f"\n Belief threshold reached ({posterior:.5f} after {elapsed:.2f}s → {decision}")
            self.report_results()
            rclpy.shutdown()

    def report_results(self):
        self.log.info("---- Bayseian Inference Results ----")
        self.log.info(f"Measurments Processed: {len(self.beliefs)}")
        self.log.info(f"Final belief probability open: {self.belief_open:.5f}")
        self.log.info(f"Example beliefs (first 10): {self.beliefs[:10]}")
        self.log.info(f"Example beliefs (last 10):  {self.beliefs[-10:]}")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    lab_3_assignment_6 = Lab_3Assignment_6()
    lab_3_assignment_6.spin()
    lab_3_assignment_6.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
