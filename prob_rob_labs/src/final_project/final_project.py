#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import quaternion_from_euler

def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def angle_mean(angles, weights):
    sin_sum = np.sum(np.sin(angles) * weights)
    cos_sum = np.sum(np.cos(angles) * weights)
    return math.atan2(sin_sum, cos_sum)

class FinalProjectNode(Node):
    def __init__(self):
        super().__init__('final_project_node')

        self.declare_parameter('filter_type', 'ekf')
        self.filter_type = self.get_parameter('filter_type').get_parameter_value().string_value.lower()
        
        if self.filter_type not in ['ekf', 'ukf']:
            self.get_logger().error(f"Invalid filter_type '{self.filter_type}'. Defaulting to EKF.")
            self.filter_type = 'ekf'
            
        self.get_logger().info(f"--- Odometry Node Started in [{self.filter_type.upper()}] mode ---")

        self.r_w = 0.033
        self.R = 0.1435     
        self.tau_v = 0.3591 
        self.tau_w = 0.446 
        self.Gv = 1.0 
        self.Gw = 1.0 

        w_peak = 0.482       # rad/s (max overshoot)
        w_steady = 0.299     # rad/s (steady state command)
        t_start = 0.574      # seconds (start time)
        t_peak = 0.704       # seconds (time to first peak)

        # Calculate Zeta and Omega_n for second order model
        PO = (w_peak - w_steady) / w_steady
        ln_po = math.log(PO)
        self.zeta = -ln_po / math.sqrt(math.pi**2 + ln_po**2)
        self.wn = math.pi / ((t_peak-t_start) * math.sqrt(1 - self.zeta**2))

        self.x = np.zeros((6, 1), dtype=float)
        self.P = np.diag([1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2])

        self.q_base = np.array([1e-4, 1e-4, 1e-4, 1e-3, 5e-3, 10.0])

        self.R_meas = np.diag([
            (0.02)**2,
            (0.02)**2,
            (0.005)**2
        ])

        self.u_v = 0.0
        self.u_w = 0.0
        self.cmd_timeout = 0.5
        self.last_cmd_time = None

        self.last_time = None

        self.n_x = 6
        self.alpha = 1e-3
        self.kappa = 0.0
        self.beta = 2.0
        self.lam = self.alpha**2 * (self.n_x + self.kappa) - self.n_x

        self.gamma = math.sqrt(self.n_x + self.lam)
        self.Wm = np.full(2 * self.n_x + 1, 0.5 / (self.n_x + self.lam))
        self.Wc = np.full(2 * self.n_x + 1, 0.5 / (self.n_x + self.lam))
        self.Wm[0] = self.lam / (self.n_x + self.lam)
        self.Wc[0] = self.lam / (self.n_x + self.lam) + (1 - self.alpha**2 + self.beta)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        self.imu_sub = Subscriber(self, Imu, '/imu')
        self.jnt_sub = Subscriber(self, JointState, '/joint_states')

        self.sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.jnt_sub], queue_size=30, slop=0.03)
        self.sync.registerCallback(self.sync_cb)

        self.pub = self.create_publisher(Odometry, '/ekf_odom', 10)

    def cmd_cb(self, msg: Twist):
        self.u_v = float(msg.linear.x)
        self.u_w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def sync_cb(self, imu_msg: Imu, jnt_msg: JointState):
        t = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
            return
        
        dt = t - self.last_time
        if dt <= 0.0 or dt > 0.5:
            self.last_time = t
            return
        self.last_time = t

        fresh = (
            self.last_cmd_time is not None and
            (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 <= self.cmd_timeout
        )
        u_v_eff = self.u_v if fresh else 0.0
        u_w_eff = self.u_w if fresh else 0.0

        if self.filter_type == 'ekf':
            self.predict_ekf(dt, u_v_eff, u_w_eff)
            self.update_ekf(imu_msg, jnt_msg)
        else:
            self.predict_ukf(dt, u_v_eff, u_w_eff)
            self.update_ukf(imu_msg, jnt_msg)

        self.publish_odom(t)

    def transition_function(self, state, dt, u_v, u_w):
        x, y, th, v, w, alpha = state.flatten()

        nx = x + v * math.cos(th) * dt
        ny = y + v * math.sin(th) * dt
        nth = wrap_pi(th + w * dt)

        a_v = 0.1 ** (dt / self.tau_v)
        nv = a_v * v + self.Gv * (1.0 - a_v) * u_v

        target_w = self.Gw * u_w
        alpha_dot = -2 * self.zeta * self.wn * alpha - (self.wn**2) * (w - target_w)
        
        nalpha = alpha + alpha_dot * dt
        nw = w + alpha * dt

        return np.array([[nx], [ny], [nth], [nv], [nw], [nalpha]])

    def measurement_model_matrix(self):
        C = np.array([
            [0, 0, 0, 1 / self.r_w,  self.R / self.r_w, 0.0],
            [0, 0, 0, 1 / self.r_w, -self.R / self.r_w, 0.0],
            [0, 0, 0, 0,            1.0,                0.0]
        ], dtype=float)
        return C
    
    def predict_ekf(self, dt, u_v, u_w):
        x_curr = self.x
        self.x = self.transition_function(x_curr, dt, u_v, u_w)

        _, _, th, v, w, alpha = x_curr.flatten()
        a_v = 0.1 ** (dt / self.tau_v)
        sin_th = math.sin(th)
        cos_th = math.cos(th)

        F = np.eye(6)
        F[0, 2] = -v * sin_th * dt   # dx/dth
        F[0, 3] = cos_th * dt        # dx/dv

        F[1, 2] = v * cos_th * dt    # dy/dth
        F[1, 3] = sin_th * dt        # dy/dv

        F[2, 4] = dt                 # dth/dw

        F[3, 3] = a_v                # dv/dv

        F[4, 4] = 1.0                # dw/dw
        F[4, 5] = dt                 # dw/dalpha

        F[5, 5] = 1.0 - 2.0 * self.zeta * self.wn * dt    # d(alpha)/d(alpha)
        F[5, 4] = -(self.wn**2) * dt                      # d(alpha)/d(w)

        Q = np.diag(self.q_base * max(dt, 1e-3))
        self.P = F @ self.P @ F.T + Q

    def update_ekf(self, imu_msg, jnt_msg):
        z = self.get_measurement_vector(imu_msg, jnt_msg)
        if z is None: return

        C = self.measurement_model_matrix()
        
        z_hat = C @ self.x
        y = z - z_hat

        S = C @ self.P @ C.T + self.R_meas
        
        K = self.P @ C.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = wrap_pi(self.x[2, 0])

        I = np.eye(6)
        self.P = (I - K @ C) @ self.P

    def compute_sigma_points(self, x, P):

        try:
            L = np.linalg.cholesky(P)
        except np.linalg.LinAlgError:
            self.get_logger().warn("UKF: Cholesky failed, adding epsilon.")
            L = np.linalg.cholesky(P + np.eye(self.n_x) * 1e-9)

        X_sig = np.zeros((self.n_x, 2 * self.n_x + 1))
        X_sig[:, 0] = x[:, 0]
        
        term = self.gamma * L
        for i in range(self.n_x):
            X_sig[:, i + 1] = x[:, 0] + term[:, i]
            X_sig[:, i + 1 + self.n_x] = x[:, 0] - term[:, i]
            
        return X_sig

    def predict_ukf(self, dt, u_v, u_w):
        X_sig = self.compute_sigma_points(self.x, self.P)
        
        X_pred_sig = np.zeros_like(X_sig)
        for i in range(2 * self.n_x + 1):
            col = X_sig[:, i:i+1]
            X_pred_sig[:, i:i+1] = self.transition_function(col, dt, u_v, u_w)

        x_pred = np.zeros((self.n_x, 1))
        
        x_pred[0] = np.sum(self.Wm * X_pred_sig[0, :]) # x
        x_pred[1] = np.sum(self.Wm * X_pred_sig[1, :]) # y
        x_pred[3] = np.sum(self.Wm * X_pred_sig[3, :]) # v
        x_pred[4] = np.sum(self.Wm * X_pred_sig[4, :]) # w
        
        x_pred[2] = angle_mean(X_pred_sig[2, :], self.Wm) # th

        P_pred = np.zeros((self.n_x, self.n_x))
        for i in range(2 * self.n_x + 1):
            diff = X_pred_sig[:, i:i+1] - x_pred
            diff[2, 0] = wrap_pi(diff[2, 0])
            P_pred += self.Wc[i] * (diff @ diff.T)
            
        Q = np.diag(self.q_base * max(dt, 1e-3))
        P_pred += Q

        self.x = x_pred
        self.P = P_pred
        self.X_pred_sig = X_pred_sig

    def update_ukf(self, imu_msg, jnt_msg):
        z = self.get_measurement_vector(imu_msg, jnt_msg)
        if z is None: return

        X_sig = self.compute_sigma_points(self.x, self.P)

        C = self.measurement_model_matrix()
        Z_sig = C @ X_sig

        z_pred = np.sum(self.Wm * Z_sig, axis=1, keepdims=True)

        S = np.zeros((3, 3))
        T = np.zeros((self.n_x, 3))
        
        for i in range(2 * self.n_x + 1):
            z_diff = Z_sig[:, i:i+1] - z_pred
            x_diff = X_sig[:, i:i+1] - self.x
            x_diff[2, 0] = wrap_pi(x_diff[2, 0])

            S += self.Wc[i] * (z_diff @ z_diff.T)
            T += self.Wc[i] * (x_diff @ z_diff.T)

        S += self.R_meas

        K = T @ np.linalg.inv(S)
        y = z - z_pred
        
        self.x = self.x + K @ y
        self.x[2, 0] = wrap_pi(self.x[2, 0])
        self.P = self.P - K @ S @ K.T
    
    def get_measurement_vector(self, imu_msg, jnt_msg):
        omega_r = 0.0
        omega_l = 0.0
        try:
            idx_r = jnt_msg.name.index('wheel_right_joint')
            idx_l = jnt_msg.name.index('wheel_left_joint')
            if len(jnt_msg.velocity) > max(idx_r, idx_l):
                omega_r = float(jnt_msg.velocity[idx_r]) 
                omega_l = float(jnt_msg.velocity[idx_l])
            else:
                return None
        except ValueError:
            return None

        w_gyro = float(imu_msg.angular_velocity.z)
        
        return np.array([[omega_r], [omega_l], [w_gyro]])

    def publish_odom(self, t_sec):
        odom = Odometry()
        odom.header.stamp.sec = int(t_sec)
        odom.header.stamp.nanosec = int((t_sec - int(t_sec)) * 1e9)
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        x, y, th, v, w, alpha = self.x.flatten()
        q = quaternion_from_euler(0.0, 0.0, th)

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)

        P = self.P
        pose_cov = np.zeros(36)
        pose_cov[0]  = P[0,0] # x
        pose_cov[7]  = P[1,1] # y
        pose_cov[35] = P[2,2] # theta
        odom.pose.covariance = pose_cov.tolist()

        twist_cov = np.zeros(36)
        twist_cov[0]  = P[3,3] # v
        twist_cov[35] = P[4,4] # w
        odom.twist.covariance = twist_cov.tolist()

        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = FinalProjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shut down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()