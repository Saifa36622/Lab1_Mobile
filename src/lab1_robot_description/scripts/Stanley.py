#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import yaml
from ament_index_python.packages import get_package_share_directory

def normalize_angle(angle):
    """Normalize angle into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

class StanleyController(Node):
    """
    A ROS2 node implementing the Stanley controller for an Ackermann-drive mobile robot.
    Subscribes to '/model_odom' (nav_msgs/Odometry) for robot state.
    Publishes to '/cmd_vel' (geometry_msgs/Twist) for velocity commands.
    This version loops over the path (closed loop) indefinitely.
    """

    def __init__(self):
        super().__init__('stanley_controller')
        
        # Declare and read parameters
        self.declare_parameter('odom', "ground_truth")
        self.odom = self.get_parameter('odom').get_parameter_value().string_value
        path = get_package_share_directory("lab1_robot_description")
        self.declare_parameter('path_file', path+'/config'+'/path.yaml')
        self.declare_parameter('gain_k', 0.2)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('wheelbase', 0.2)  # L
        self.declare_parameter('loop_path', True) # If True, repeat path (closed loop)

        # Get parameter values
        path_file     = self.get_parameter('path_file').value
        self.k        = float(self.get_parameter('gain_k').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.L        = float(self.get_parameter('wheelbase').value)
        self.loop_path= bool(self.get_parameter('loop_path').value)
        self.check = 0
        # Load waypoints from YAML file
        self.waypoints = []
        if path_file:
            try:
                with open(path_file, 'r') as file:
                    data = yaml.safe_load(file)
                    # The YAML is expected to contain a list or dict of waypoints
                    raw_points = data.get('waypoints', data) if isinstance(data, dict) else data
                    for pt in raw_points:
                        # Either {x: ..., y: ...} or [x, y]
                        if isinstance(pt, dict) and 'x' in pt and 'y' in pt:
                            x = float(pt['x']); y = float(pt['y'])
                        elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                            x = float(pt[0]);    y = float(pt[1])
                        else:
                            continue
                        self.waypoints.append((x, y))
                if len(self.waypoints) == 0:
                    self.get_logger().error("No waypoints found in the path file.")
                else:
                    self.get_logger().info(
                        f"Loaded {len(self.waypoints)} waypoints from {path_file}"
                    )
            except Exception as e:
                self.get_logger().error(f"Failed to load path file: {e}")
        else:
            self.get_logger().error("Parameter 'path_file' is not provided. Exiting.")
            rclpy.shutdown()
            return

        # Internal states
        self.x = None
        self.y = None
        self.yaw = None
        self.velocity = 0.0

        # For closed loop: just keep cycling through the same waypoints
        self.target_index = 0

        # ROS2 subscribers and publishers
        self.odom_sub = self.create_subscription(Odometry, "/model_odom", self.odom_callback, 10)
        if self.odom != "ground_truth":
            self.odom_sub = self.create_subscription(PoseStamped, "/"+self.odom+"/ekf_pose", self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Stanley controller node started (closed-loop mode).")

    def odom_callback(self, msg: Odometry):
        """Callback to update robot pose from /model_odom."""
        if self.odom == "ground_truth":
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            # Convert quaternion to yaw
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self.x = pos.x
            self.y = pos.y
            self.yaw = yaw
        self.velocity = msg.twist.twist.linear.x
    
    def pose_callback(self,msg:PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.x = pos.x
        self.y = pos.y
        self.yaw = yaw

    def calc_target_index(self):
        """
        Compute the index of the nearest waypoint to the front axle:
          front_x = x + L*cos(yaw)
          front_y = y + L*sin(yaw)
        Search for the closest waypoint to (front_x, front_y).
        """
        if not self.waypoints:
            return 0, 0.0

        fx = self.x + self.L * math.cos(self.yaw)
        fy = self.y + self.L * math.sin(self.yaw)

        min_dist = float('inf')
        nearest_idx = 0
        for i, (wx, wy) in enumerate(self.waypoints):
            dx = fx - wx
            dy = fy - wy
            dist_sq = dx*dx + dy*dy
            if dist_sq < min_dist:
                min_dist = dist_sq
                nearest_idx = i

        return nearest_idx, math.sqrt(min_dist)

    def control_loop(self):
        """Periodic control loop for Stanley steering in closed loop."""
        # Wait for odometry
        if self.x is None or not self.waypoints:
            return

        # 1) Find nearest waypoint index from the front axle
        self.target_index, front_dist = self.calc_target_index()

        # 2) Optionally, you can re-order the target_index to ensure it is at or after the last known index
        # But in a closed loop, it's fine to jump around as we circle

        # 3) Next waypoint for heading
        next_idx = (self.target_index + 1) % len(self.waypoints) \
                   if self.loop_path else min(self.target_index + 1, len(self.waypoints)-1)

        # vector from waypoint[target_index] to [next_idx]
        dx = self.waypoints[next_idx][0] - self.waypoints[self.target_index][0]
        dy = self.waypoints[next_idx][1] - self.waypoints[self.target_index][1]
        path_heading = math.atan2(dy, dx)

        # 4) heading error
        heading_error = normalize_angle(path_heading - self.yaw)

        # 5) cross-track error = project offset onto path normal
        fx = self.x + self.L * math.cos(self.yaw)
        fy = self.y + self.L * math.sin(self.yaw)
        dx_fx = self.waypoints[self.target_index][0] - fx
        dy_fx = self.waypoints[self.target_index][1] - fy
        # normal to path_heading is: (-sin(path_heading), cos(path_heading))
        cross_track = dx_fx * (-math.sin(path_heading)) + dy_fx * (math.cos(path_heading))

        # 6) Stanley formula: delta = heading_error + arctan(k * cross_track / speed)
        speed = max(0.001, abs(self.velocity))  # avoid div-by-zero
        delta = heading_error + math.atan2(self.k * cross_track, speed)
        if self.target_index > 100 :
            self.check = 1
        if (self.target_index >= 721 or self.target_index < 100 ) and self.check == 1 :
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("End of the track")

            
            rclpy.shutdown()
            

        # 7) Publish ackermann-like twist
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = delta / self.L
        self.cmd_pub.publish(twist)
        
        # Debug logs
        self.get_logger().info(
            f"idx={self.target_index}, " +
            f"front_dist={front_dist:.2f}, " +
            f"heading_error(deg)={math.degrees(heading_error):.2f}, " +
            f"cross_track={cross_track:.2f}, " +
            f"delta(deg)={math.degrees(delta):.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
