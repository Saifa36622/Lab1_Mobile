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

class PurePursuitController(Node):
    """
    A ROS2 node implementing a Pure Pursuit controller for an Ackermann-drive mobile robot.
    Subscribes to '/model_odom' (nav_msgs/Odometry) for robot state.
    Publishes to '/cmd_vel' (geometry_msgs/Twist) for velocity commands.

    Only one stop condition: if waypoint index >= 100 => stop the robot.
    """

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Declare parameters
        self.declare_parameter('odom', "ground_truth")
        self.odom = self.get_parameter('odom').get_parameter_value().string_value
        path = get_package_share_directory("lab1_robot_description")
        self.declare_parameter('path_file', path+'/config'+'/path.yaml')
        self.declare_parameter('k', 0.0)       # look-forward distance gain
        self.declare_parameter('Lfc', 0.2)     # base look-ahead distance
        self.declare_parameter('Kp', 0.0)      # speed-PID gain
        self.declare_parameter('dt', 0.1)      # time step
        self.declare_parameter('WB', 0.2)      # wheelbase
        self.declare_parameter('target_speed', 1.0)

        # Read parameters
        path_file   = self.get_parameter('path_file').value
        self.k      = float(self.get_parameter('k').value)
        self.Lfc    = float(self.get_parameter('Lfc').value)
        self.Kp     = float(self.get_parameter('Kp').value)
        self.dt     = float(self.get_parameter('dt').value)
        self.WB     = float(self.get_parameter('WB').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.check = 0
        # Load waypoints
        self.waypoints = []
        if path_file:
            try:
                with open(path_file, 'r') as file:
                    data = yaml.safe_load(file)
                    # The YAML might be a list or a dict with 'waypoints'
                    raw_points = data.get('waypoints', data) if isinstance(data, dict) else data
                    for pt in raw_points:
                        if isinstance(pt, dict) and 'x' in pt and 'y' in pt:
                            x = float(pt['x']); y = float(pt['y'])
                        elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                            x = float(pt[0]);   y = float(pt[1])
                        else:
                            continue
                        self.waypoints.append((x, y))
                if len(self.waypoints) == 0:
                    self.get_logger().error("No waypoints found in path file.")
                else:
                    self.get_logger().info(
                        f"Loaded {len(self.waypoints)} waypoints from {path_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to load path file: {e}")
        else:
            self.get_logger().error("Parameter 'path_file' not provided; shutting down.")
            rclpy.shutdown()
            return

        # Internal states
        self.index = 0   # current waypoint index
        self.stop_flag = False

        # Robot state (rear-axle model)
        self.x, self.y, self.yaw = None, None, None
        self.v = 0.0

        # ROS2 sub/pub
        self.odom_sub = self.create_subscription(Odometry, "/model_odom", self.odom_callback, 10)
        if self.odom != "ground_truth":
            self.odom_sub = self.create_subscription(PoseStamped, "/"+self.odom+"/ekf_pose", self.pose_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("PurePursuit node started (stop ONLY at index >= 100).")

    def odom_callback(self, msg):
        """Extract robot pose and velocity from /model_odom."""
        if self.odom == "ground_truth":
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            # Quaternion → Yaw
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y**2 + ori.z**2)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self.x   = pos.x
            self.y   = pos.y
            self.yaw = normalize_angle(yaw)
        self.v   = msg.twist.twist.linear.x

    def pose_callback(self,msg:PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        # Quaternion → Yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y**2 + ori.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.x   = pos.x
        self.y   = pos.y
        self.yaw = normalize_angle(yaw)

    def control_loop(self):
        """Compute control commands and publish /cmd_vel."""
        # Wait for at least one odom reading and at least one waypoint
        if self.x is None or len(self.waypoints) == 0:
            return

        # If we've flagged to stop, keep robot stationary
        if self.stop_flag:
            self.cmd_pub.publish(Twist())
            return

        # ===========================
        # Sole Stop Condition: index >= 100
        # ===========================
        if self.index >= 100 :
            self.check = 1
        if self.index < 100 and self.check == 1:
            self.get_logger().info(
                f"Waypoint index >= 100 => stopping robot.")
            self.stop_flag = True
            self.cmd_pub.publish(Twist())
            return
        
        

        # Pure Pursuit: 1) build lookahead distance
        Lf = self.k * abs(self.v) + self.Lfc

        # 2) find the first waypoint that is farther than Lf
        target_ind = self.index
        while True:
            if target_ind >= len(self.waypoints) - 1:
                break
            tx, ty = self.waypoints[target_ind]
            d = math.hypot(tx - self.x, ty - self.y)
            if d > Lf:
                break
            target_ind += 1

        if target_ind >= len(self.waypoints):
            target_ind = len(self.waypoints) - 1

        # 3) compute steering
        tx, ty = self.waypoints[target_ind]
        alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw
        alpha = normalize_angle(alpha)
        delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)

        # 4) speed control if Kp>0
        if self.Kp > 0:
            error_v = self.target_speed - abs(self.v)
            a = self.Kp * error_v
            new_v = self.v + a * self.dt
            # clamp speed
            new_v = max(-self.target_speed, min(self.target_speed, new_v))
        else:
            new_v = self.target_speed

        if target_ind == 721 :
            self.get_logger().info(
                f"Waypoint index >= 100 => stopping robot.")
            self.stop_flag = True
            self.cmd_pub.publish(Twist())
        # 5) publish velocity
        twist = Twist()
        twist.linear.x  = float(new_v)
        twist.angular.z = float(delta)
        self.cmd_pub.publish(twist)

        # 6) increment index to reflect “progress”
        # (Simple approach: if we’re within some small distance of current WP)
        # But the user wants no distance-based stop, so let's do a simple index++ each cycle
        self.index += 1

        # Debug
        self.get_logger().info(
            f"idx={self.index}, Lf={Lf:.2f}, alpha(deg)={math.degrees(alpha):.1f}, "
            f"delta(deg)={math.degrees(delta):.1f}, speed={new_v:.2f}, target_ind={target_ind}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
