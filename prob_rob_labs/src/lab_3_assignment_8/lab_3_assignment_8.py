#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Twist

heartbeat_period = 1.0         
feature_threshold = 238.0       
bayes_threshold = 0.99          

P_z_o_x_o = 0.977               
P_z_c_x_o = 1.0 - P_z_o_x_o     
P_z_c_x_c = 0.948               
P_z_o_x_c = 1.0 - P_z_c_x_c     

P_o_given_push = 0.6875  
P_open_stays_open_on_push = 1.0

push_every_secs = 1.0     

move_forward_speed = 2.0 
move_forward_secs = 3.0   


class Lab3Assignment8(Node):
    def __init__(self):
        super().__init__('lab_3_assignment_8')
        self.log = self.get_logger()

        self.feature_sub = self.create_subscription(
            Float64, '/feature_mean', self.feature_callback, 10
        )
        self.pub_open = self.create_publisher(Empty, '/door_open', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.feature_val = None
        self.have_new_feature = False
        self.belief_open = 0.5
        self.beliefs = [self.belief_open]

        self.last_push_time = 0.0
        self.moving = False
        self.move_start = None

        self.committed = False
        self.move_timer = None
        self.move_end_time = None

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.start_time = time.time()
        self.log.info("Assignment 8 node started: controller + Bayes filter")

    def feature_callback(self, msg: Float64):
        if self.committed:
            return
        self.feature_val = float(msg.data)
        self.have_new_feature = True

    def predict_with_push(self, bel_open_prev: float) -> float:
        """
        bel_t|t-1 = P(x_t=open | push) =
            bel_prev * P(open->open | push) + (1-bel_prev) * P(closed->open | push)
        """
        term_open = bel_open_prev * P_open_stays_open_on_push
        term_closed = (1.0 - bel_open_prev) * P_o_given_push
        return max(0.0, min(1.0, term_open + term_closed))

    def update_with_measurement(self, bel_open_pred: float, z_is_open: bool) -> float:
        """
        bel_t = η P(z|x=open) bel_pred
        """
        if z_is_open:
            p_z_given_open = P_z_o_x_o
            p_z_given_closed = P_z_o_x_c
        else:
            p_z_given_open = P_z_c_x_o
            p_z_given_closed = P_z_c_x_c

        num = p_z_given_open * bel_open_pred
        den = num + p_z_given_closed * (1.0 - bel_open_pred)
        if den <= 0.0:
            return bel_open_pred 
        return max(0.0, min(1.0, num / den))

    def publish_open_command(self):
        if self.committed:
            return
        self.pub_open.publish(Empty())

    def start_motion(self):
        if self.committed:
            return
        self.committed = True
        self.moving = True
        self.move_start = time.time()
        self.move_end_time = self.move_start + move_forward_secs

        try:
            self.timer.cancel()
        except Exception:
            pass

        try:
            self.destroy_subscription(self.feature_sub)
            self.feature_sub = None
        except Exception:
            pass

        self.log.info("Belief >= threshold → COMMITTING: stop pushes/measurements, MOVING FORWARD now")

        self.move_timer = self.create_timer(0.1, self.move_tick)

        twist = Twist()
        twist.linear.x = move_forward_speed
        self.cmd_pub.publish(twist)

    def move_tick(self):
        now = time.time()
        if now < self.move_end_time:
            twist = Twist()
            twist.linear.x = move_forward_speed
            self.cmd_pub.publish(twist)
        else:
            try:
                self.move_timer.cancel()
            except Exception:
                pass
            self.cmd_pub.publish(Twist())  
            self.report_results(final=True)
            rclpy.shutdown()

    def heartbeat(self):
        if self.committed:
            return 

        now = time.time()

        if (now - self.last_push_time) >= push_every_secs:
            self.publish_open_command()
            self.last_push_time = now
            bel_pred = self.predict_with_push(self.belief_open)
        else:
            bel_pred = self.belief_open

        if self.have_new_feature and (self.feature_val is not None):
            z_is_open = (self.feature_val <= feature_threshold)
            bel_post = self.update_with_measurement(bel_pred, z_is_open)
            self.belief_open = bel_post
            self.beliefs.append(self.belief_open)
            self.have_new_feature = False

            z_str = "open" if z_is_open else "closed"
            self.log.info(
                f"feature_mean={self.feature_val:.2f} → z={z_str} | "
                f"belief_open={self.belief_open:.5f}"
            )
        else:
            self.belief_open = bel_pred

        if self.belief_open >= bayes_threshold:
            self.start_motion()

    def report_results(self, final=False):
        hdr = "FINAL" if final else "INTERIM"
        elapsed = time.time() - self.start_time
        self.log.info(f"---- {hdr} Bayes Estimator Report ----")
        self.log.info(f"Elapsed: {elapsed:.2f}s | Steps: {len(self.beliefs)}")
        self.log.info(f"Belief(open): {self.belief_open:.5f}")
        tail = self.beliefs[-10:] if len(self.beliefs) > 10 else self.beliefs[:]
        self.log.info(f"Belief tail (up to 10): {['%.3f'%b for b in tail]}")


def main():
    rclpy.init()
    node = Lab3Assignment8()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
