#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import yaml

import numpy as np

# Measurement noise covariance (2x2)
R = np.array([[0.1, 0.0],
              [0.0, 0.1]])

# Dynamic model and Jacobian for the prediction step
def dynamic_model(x, u, dt):
    """
    คำนวณ state ใหม่จาก state ปัจจุบันด้วยสมการ:
      xₖ₊₁ = xₖ + (vₖ*cos(ϕₖ)*dt)
      yₖ₊₁ = yₖ + (vₖ*sin(ϕₖ)*dt)
      ϕₖ₊₁ = ϕₖ + ωₖ*dt
    """
    x_new = np.zeros((3,1))
    x_new[0][0] = x[0][0] + (u[0][0]*np.cos(x[2][0])*dt)
    x_new[1][0] = x[1][0] + (u[0][0]*np.sin(x[2][0])*dt)
    x_new[2][0] = x[2][0] + (u[1][0]*dt)
    return x_new

def ekf_predict(xEst, PEst, u, dt, Q):
    F = np.array([[1, 0, -u[0][0]*np.sin(xEst[2][0])*dt],
                  [0, 1, u[0][0]*np.cos(xEst[2][0])*dt],
                  [0, 0, 1]])
    xPred = dynamic_model(xEst, u, dt)
    PPred = F @ PEst @ F.T + Q
    return xPred, PPred

def ekf_update(xEst, PEst, z, R):
    H = np.zeros((2, 3))
    H[0:2, 0:2] = np.eye(2)
    zPred = H @ xEst
    y = z - zPred
    S = H @ PEst @ H.T + R
    K = PEst @ H.T @ np.linalg.inv(S)
    xEst_new = xEst + K @ y
    PEst_new = (np.eye(3) - K @ H) @ PEst
    return xEst_new, PEst_new

class EKFFullNode(Node):
    def __init__(self):
        super().__init__('ekf_full_node')

        self.declare_parameter('pose', [0.0, 0.0, 0.0])
        self.start_pose = self.get_parameter('pose').get_parameter_value().double_array_value
        self.declare_parameter('noise', [0.0, 0.0])
        self.noise = self.get_parameter('noise').get_parameter_value().double_array_value

        # Process noise covariance Q (3x3)
        self.Q = np.array([[self.noise[0]**2, 0.0, 0.0],
                           [0.0, self.noise[0]**2, 0.0],
                           [0.0, 0.0, self.noise[1]**2]])

        self.dt = 0.1
        self.last_time = self.get_clock().now()
        self.xEst = np.array([[self.start_pose[0]],[self.start_pose[1]],[self.start_pose[2]]])
        self.PEst = np.eye(3)
        self.new_u = False
        self.z_gps = None
        self.new_gps = False
        self.u = np.zeros((2,1))
        # self.ekf_file = open(self.get_namespace()[1:]+"_ekf.yaml", "a")

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.gps_sub = self.create_subscription(PoseStamped, '/gps', self.gps_callback, 10)
        self.ekf_pub = self.create_publisher(PoseStamped, 'ekf_pose', 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def odom_callback(self, msg:Odometry):
        vx = msg.twist.twist.linear.x
        omega_z = msg.twist.twist.angular.z
        self.u = np.array([[vx], [omega_z]])
        self.new_u = True
        
    def gps_callback(self, msg:PoseStamped):
        px = msg.pose.position.x
        py = msg.pose.position.y
        self.z_gps = np.array([[px], [py]])
        self.new_gps = True
        
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time
        
        if self.new_u and self.new_gps and self.z_gps is not None:
            # Prediction step
            self.xEst, self.PEst = ekf_predict(self.xEst, self.PEst, self.u, dt, self.Q)
            self.new_u = False
        
            # Update step if new measurements are available
            self.xEst, self.PEst = ekf_update(self.xEst, self.PEst, self.z_gps, R)
            self.new_gps = False
        
        self.publish_estimate()
        
    def publish_estimate(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = self.xEst[0][0]
        msg.pose.position.y = self.xEst[1][0]
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.xEst[2][0])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.ekf_pub.publish(msg)

        # # Convert the timestamp to seconds (including nanoseconds).
        # stamp = self.get_clock().now().to_msg()
        # timestamp_sec = stamp.sec + stamp.nanosec * 1e-9

        #  # Create a dictionary to represent the odometry message.
        # data = {
        #     'timestamp': timestamp_sec,
        #     'position': {
        #         'x': float(self.xEst[0][0]),
        #         'y': float(self.xEst[1][0]),
        #         'z': 0.0
        #     },
        #     'orientation': {
        #         'x': 0.0,
        #         'y': 0.0,
        #         'z': float(self.xEst[2][0])
        #     }
        # }

        # # Append the new data as a YAML document.
        # self.ekf_file.write("---\n")
        # yaml.dump(data, self.ekf_file)
        # self.ekf_file.flush()

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFFullNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
