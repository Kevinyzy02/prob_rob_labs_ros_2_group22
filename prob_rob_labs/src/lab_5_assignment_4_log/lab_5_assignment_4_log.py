#!/usr/bin/env python3
import csv, os, time, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

class A4Logger(Node):

    def __init__(self):
        super().__init__('a4_logger')
        self.declare_parameter('outfile', 'a4_measurements.csv')
        self.outfile = self.get_parameter('outfile').value

        self.color = None
        self.d_meas = None
        self.theta_meas = None
        self.d_err = None
        self.theta_err = None

        self.create_subscription(String,  '/landmark_active/color',              self.cb_color,    10)
        self.create_subscription(Float64, '/measurement/measured_distance',      self.cb_d_meas,   10)
        self.create_subscription(Float64, '/measurement/measured_bearing',       self.cb_th_meas,  10)
        self.create_subscription(Float64, '/measurement_error/distance',         self.cb_d_err,    10)
        self.create_subscription(Float64, '/measurement_error/bearing',          self.cb_th_err,   10)

        newfile = not os.path.exists(self.outfile)
        self.fh = open(self.outfile, 'a', newline='')
        self.wr = csv.writer(self.fh)
        if newfile:
            self.wr.writerow(['t', 'color', 'd_meas_m', 'theta_meas_rad', 'd_err_m', 'theta_err_rad', 'h_pix'])

        self.create_timer(1.0, self.flush)
        self.get_logger().info(f'Logging to {os.path.abspath(self.outfile)}')

        self.create_timer(0.05, self.try_write_row)

    def cb_color(self, msg):      self.color = msg.data
    def cb_d_meas(self, msg):     self.d_meas = float(msg.data)
    def cb_th_meas(self, msg):    self.theta_meas = float(msg.data)
    def cb_d_err(self, msg):      self.d_err = float(msg.data)
    def cb_th_err(self, msg):     self.theta_err = float(msg.data)

    def try_write_row(self):
        if None in (self.color, self.d_meas, self.theta_meas, self.d_err, self.theta_err):
            return
        t = time.time()
        self.wr.writerow([f'{t:.3f}', self.color, f'{self.d_meas:.6f}', f'{self.theta_meas:.6f}',
                          f'{self.d_err:.6f}', f'{self.theta_err:.6f}'])

    def flush(self):
        try:
            self.fh.flush()
            os.fsync(self.fh.fileno())
        except Exception:
            pass

def main():
    rclpy.init()
    node = A4Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.fh.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
