#!/usr/bin/python3

import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

class AckermannPIDController(Node):
    def __init__(self):
        super().__init__('ackermann_pid_controller')

        # Declare and get parameters
        self.declare_parameter('odom', "ground_truth")
        self.odom = self.get_parameter('odom').get_parameter_value().string_value
        path = get_package_share_directory("lab1_robot_description")
        self.declare_parameter('path_file', path+'/config'+'/path.yaml')
        self.declare_parameter('Kp_linear', 1.0)
        self.declare_parameter('Ki_linear', 0.0)
        self.declare_parameter('Kd_linear', 0.0)
        self.declare_parameter('Kp_angular', 1.0)
        self.declare_parameter('Ki_angular', 0.0)
        self.declare_parameter('Kd_angular', 0.0)
        
        path_file = self.get_parameter('path_file').get_parameter_value().string_value

        # Load waypoints from the YAML file
        try:
            with open(path_file, 'r') as file:
                data = yaml.safe_load(file)  # Load YAML as a list

            if isinstance(data, list):  # Direct list format
                self.waypoints = [(pt['x'], pt['y'], pt.get('yaw', 0.0)) for pt in data]
            else:
                raise ValueError("Unexpected YAML format. Expected a list of waypoints.")

            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from {path_file}: {e}")
            self.waypoints = []

        # PID Gain Values
        self.Kp_lin = self.get_parameter('Kp_linear').get_parameter_value().double_value
        self.Ki_lin = self.get_parameter('Ki_linear').get_parameter_value().double_value
        self.Kd_lin = self.get_parameter('Kd_linear').get_parameter_value().double_value
        self.Kp_ang = self.get_parameter('Kp_angular').get_parameter_value().double_value
        self.Ki_ang = self.get_parameter('Ki_angular').get_parameter_value().double_value
        self.Kd_ang = self.get_parameter('Kd_angular').get_parameter_value().double_value

        # Initialize State
        self.current_idx = 0
        self.x = self.y = self.yaw = 0.0
        self.current_speed = 0.0
        self.dist_threshold = 0.5  # Distance threshold to switch waypoints

        # PID error accumulators
        self.linear_error_int = 0.0
        self.linear_error_prev = 0.0
        self.angular_error_int = 0.0
        self.angular_error_prev = 0.0
        self.last_time = self.get_clock().now()

        # ROS Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, "/model_odom", self.odom_callback, 10)
        if self.odom != "ground_truth":
            self.odom_sub = self.create_subscription(PoseStamped, "/"+self.odom+"/ekf_pose", self.pose_callback, 10)
            
        # Timer for Control Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Ackermann PID Controller Node has started.")

    def odom_callback(self, msg: Odometry):
        """ Updates the current position and orientation from odometry. """
        if self.odom == "ground_truth":
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y

            # Convert quaternion to yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Compute current linear speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def pose_callback(self,msg:PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """ Main control loop to track waypoints. """
        if not self.waypoints or self.current_idx >= len(self.waypoints):
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Get current target waypoint
        target_x, target_y, target_yaw = self.waypoints[self.current_idx]

        # Compute distance to waypoint
        dx = target_x - self.x
        dy = target_y - self.y
        distance_error = math.sqrt(dx ** 2 + dy ** 2)

        # If close enough, move to the next waypoint
        while distance_error < self.dist_threshold:
            if self.current_idx < len(self.waypoints) - 1:
                self.current_idx += 1
                self.get_logger().info(f"Reached waypoint {self.current_idx}, moving to next.")
                return
            else:
                # Stop if at final waypoint
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info("Final waypoint reached. Stopping.")
                return

        # Compute desired heading
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normalize to [-pi, pi]

        # PID Control for Linear Velocity
        desired_speed = 1.0  # Default speed
        if self.current_idx == len(self.waypoints) - 1 and distance_error < 1.0:
            desired_speed *= distance_error  # Slow down near final waypoint

        speed_error = desired_speed - self.current_speed
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        dt = max(dt, 1e-6)  # Prevent division by zero

        # Compute PID for linear velocity
        self.linear_error_int += speed_error * dt
        linear_error_deriv = (speed_error - self.linear_error_prev) / dt
        output_speed = (self.Kp_lin * speed_error +
                        self.Ki_lin * self.linear_error_int +
                        self.Kd_lin * linear_error_deriv)
        self.linear_error_prev = speed_error

        # PID Control for Angular Velocity
        yaw_error = heading_error
        self.angular_error_int += yaw_error * dt
        angular_error_deriv = (yaw_error - self.angular_error_prev) / dt
        output_yaw_rate = (self.Kp_ang * yaw_error +
                           self.Ki_ang * self.angular_error_int +
                           self.Kd_ang * angular_error_deriv)
        self.angular_error_prev = yaw_error
        self.last_time = current_time

        # Publish Twist Command
        twist = Twist()
        twist.linear.x = output_speed
        twist.angular.z = output_yaw_rate
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = AckermannPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
