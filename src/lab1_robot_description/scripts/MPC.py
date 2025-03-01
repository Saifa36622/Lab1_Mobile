#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import yaml
import cvxpy
import math
from ament_index_python.packages import get_package_share_directory

##############################################
# Adapted from Atsushi Sakai's iterative MPC #
##############################################

# MPC dimensions
NX = 4      # x = [pos_x, pos_y, velocity, yaw]
NU = 2      # u = [accel, steer]
T  = 5      # Horizon length (number of steps)

# Weights/Costs
R  = np.diag([0.01, 0.01])    # Input cost
Rd = np.diag([0.01, 1.0])     # Input difference cost
Q  = np.diag([1.0, 1.0, 0.5, 0.5])  # State cost
Qf = Q                        # Terminal state cost

# Iterative params
MAX_ITER = 3     # Max iteration in iterative linearization
DU_TH    = 0.1   # Convergence threshold

# Vehicle parameters
WB = 0.2         # Wheelbase [m]
MAX_STEER = np.deg2rad(40.0)
MAX_DSTEER = np.deg2rad(30.0)
MAX_SPEED = 0.5
MIN_SPEED = -0.5
MAX_ACCEL = 1.0
DT = 0.1         # Timestep, also used for the linear model tick

# For nearest index search
N_IND_SEARCH = 10

##############################################
# Helper functions
##############################################
def pi_2_pi(angle):
    """Normalize angle into [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def get_linear_model_matrix(v, phi, delta):
    """
    Linearized motion model around (v, phi, delta).
    A shape: (4,4), B shape: (4,2), C shape: (4,)
    """
    A = np.eye(NX)
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] =  DT * v * math.cos(phi)
    A[3, 2] =  DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * (math.cos(delta) ** 2))

    C = np.zeros(NX)
    # The reference code includes small terms in C from partial derivatives:
    # C[0] = DT * v * sin(phi) * phi  (approx, omitted in some linearizations)
    # etc. We can keep them zero to reduce complexity if desired. 
    # Below is the original reference if we want them:
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * (math.cos(delta)**2))

    return A, B, C

def update_motion(x, u):
    """
    Update the real state with the chosen control (acc, steer).
    x: [x, y, v, yaw]
    u: [a, delta]
    """
    a, delta = u
    # Clip delta
    delta = max(-MAX_STEER, min(delta, MAX_STEER))
    # Integrate with DT
    next_x = x[0] + x[2] * math.cos(x[3]) * DT
    next_y = x[1] + x[2] * math.sin(x[3]) * DT
    next_v = x[2] + a * DT
    next_yaw = x[3] + x[2] / WB * math.tan(delta) * DT
    # Clip
    next_v = max(MIN_SPEED, min(MAX_SPEED, next_v))
    return np.array([next_x, next_y, next_v, pi_2_pi(next_yaw)])

def calc_nearest_index(state, waypoints, pind):
    """
    Search for the nearest index in waypoints starting from pind.
    waypoints is a list of (x_i, y_i).
    """
    best_ind = pind
    best_dist = float('inf')
    for i in range(pind, min(pind + N_IND_SEARCH, len(waypoints))):
        dx = state[0] - waypoints[i][0]
        dy = state[1] - waypoints[i][1]
        dist = dx*dx + dy*dy
        if dist < best_dist:
            best_dist = dist
            best_ind = i
    best_dist = math.sqrt(best_dist)
    return best_ind, best_dist

def build_reference(state, waypoints, target_speed, start_idx):
    """
    Build a local reference trajectory xref, dref over T steps. 
    xref shape: (NX, T+1), dref shape: (1, T+1)
    We'll keep reference steer = 0. We assume a constant target_speed for simplicity.
    """
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))  # desired steering
    n_wp = len(waypoints)

    idx, _ = calc_nearest_index(state, waypoints, start_idx)
    # We'll just set a constant speed reference
    for i in range(T + 1):
        ref_idx = min(idx + i, n_wp - 1)
        xref[0, i] = waypoints[ref_idx][0]  # x
        xref[1, i] = waypoints[ref_idx][1]  # y
        xref[2, i] = target_speed          # v
        # We'll approximate yaw by direction from the previous waypoint
        if ref_idx > 0:
            dx = waypoints[ref_idx][0] - waypoints[ref_idx-1][0]
            dy = waypoints[ref_idx][1] - waypoints[ref_idx-1][1]
            yaw_ref = math.atan2(dy, dx)
        else:
            yaw_ref = 0.0
        xref[3, i] = yaw_ref
        dref[0, i] = 0.0

    return xref, dref, idx

def linear_mpc_control(xref, xbar, x0, dref):
    """
    One-step linear MPC control. 
    xref, xbar: shape (NX, T+1)
    x0: [x, y, v, yaw]
    dref: shape (1, T+1) => desired steer angles
    returns: oa, odelta, predicted trajectory states
    """
    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0
    constraints = []
    # initial condition
    constraints += [x[:, 0] == x0]

    for t in range(T):
        # input cost
        cost += cvxpy.quad_form(u[:, t], R)
        # state tracking cost
        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        # system dynamics
        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t+1] == A @ x[:, t] + B @ u[:, t] + C]

        # input difference cost
        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t+1] - u[:, t], Rd)
            # steering rate constraint
            constraints += [
                cvxpy.abs(u[1, t+1] - u[1, t]) <= MAX_DSTEER * DT
            ]

    # terminal cost
    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    # constraints
    # state bounds
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    # input bounds
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    # solve
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.OSQP, verbose=False)

    if prob.status not in [cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE]:
        return None, None, None

    ox = x.value[0, :]
    oy = x.value[1, :]
    ov = x.value[2, :]
    oyaw = x.value[3, :]

    oa = u.value[0, :]     # accel
    odelta = u.value[1, :] # steer

    return oa, odelta, (ox, oy, ov, oyaw)

def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    Iterative linearization for T steps.
    xref, dref: shape (NX, T+1), (1, T+1)
    x0: current state [x, y, v, yaw]
    oa, od: initial guesses (acc, steer) for T steps
    returns: final optimized (a, delta), predicted trajectory
    """
    if oa is None or od is None:
        oa = np.zeros(T)
        od = np.zeros(T)

    # predicted trajectory for linearization
    for _ in range(MAX_ITER):
        # 1) predict xbar using the current guesses
        xbar = predict_trajectory(x0, oa, od)
        # 2) run one-step linear mpc
        new_oa, new_od, pred_traj = linear_mpc_control(xref, xbar, x0, dref)
        if new_oa is None or new_od is None:
            # fail
            break
        # check convergence
        du = np.sum(np.abs(new_oa - oa)) + np.sum(np.abs(new_od - od))
        oa, od = new_oa, new_od
        if du < DU_TH:
            break

    return oa, od, pred_traj

def predict_trajectory(x0, oa, od):
    """
    Forward-simulate from x0 for T steps using the given (acc, steer) = (oa, od).
    returns xbar shape: (NX, T+1)
    """
    xbar = np.zeros((NX, T + 1))
    xbar[:, 0] = x0
    state = x0.copy()
    for i in range(T):
        a_i = oa[i]
        d_i = od[i]
        state = update_motion(state, [a_i, d_i])
        xbar[:, i+1] = state
    return xbar

##############################################

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # Parameters
        self.declare_parameter('odom', "ground_truth")
        self.odom = self.get_parameter('odom').get_parameter_value().string_value
        path = get_package_share_directory("lab1_robot_description")
        self.declare_parameter('path_file', path+'/config'+'/path.yaml')
        self.declare_parameter('target_speed', 0.3)
        self.declare_parameter('goal_tolerance', 0.2)

        # Access parameters
        path_file = self.get_parameter('path_file').value
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # Load path (waypoints)
        self.waypoints = self.load_waypoints(path_file)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

        # Robot state
        self.x = None
        self.y = None
        self.yaw = None
        self.v = 0.0

        # Index for nearest waypoint
        self.nearest_ind = 0
        self.goal_reached = False

        # MPC memory (previous solution)
        self.oa = None    # T accelerations
        self.od = None    # T steering angles

        # ROS
        if self.odom == "ground_truth":
            self.odom_sub = self.create_subscription(Odometry, "/model_odom", self.odom_callback, 10)
        else:
            self.odom_sub = self.create_subscription(PoseStamped, "/"+self.odom+"/ekf_pose", self.odom_callback, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(DT, self.control_loop)
        self.get_logger().info("Iterative MPC node started.")

    def load_waypoints(self, path_file):
        """Load (x,y) waypoints from path.yaml."""
        try:
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
            # If the YAML is just a list of dicts with x,y
            waypoints = [(pt['x'], pt['y']) for pt in data]
            return waypoints
        except Exception as e:
            self.get_logger().error(f"Failed to load {path_file}: {e}")
            return []

    def odom_callback(self, msg):
        """Extract pose from odometry."""
        if self.odom == "ground_truth":
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
        else:
            pos = msg.pose.position
            ori = msg.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.x = pos.x
        self.y = pos.y
        self.yaw = yaw

    def control_loop(self):
        """Periodic MPC cycle."""
        if self.x is None or self.goal_reached or not self.waypoints:
            return

        # Build current state
        current_state = np.array([self.x, self.y, self.v, self.yaw])

        # Check goal
        dx = self.waypoints[-1][0] - self.x
        dy = self.waypoints[-1][1] - self.y
        dist_goal = math.hypot(dx, dy)
        if dist_goal < self.goal_tolerance:
            self.goal_reached = True
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info("Goal Reached!")
            return

        # Build local reference around nearest index
        xref, dref, self.nearest_ind = build_reference(
            current_state,
            self.waypoints,
            self.target_speed,
            self.nearest_ind
        )

        # Solve iterative linear MPC
        new_oa, new_od, pred_traj = iterative_linear_mpc_control(
            xref, current_state, dref,
            self.oa, self.od
        )

        if new_oa is None or new_od is None:
            # MPC failed
            self.get_logger().warn("MPC failed. Setting velocity=0.0")
            self.publish_velocity(0.0, 0.0)
            return

        # Use only the first step from the solution
        a_cmd = new_oa[0]
        d_cmd = new_od[0]

        # Update memory
        self.oa = new_oa
        self.od = new_od

        # Update velocity
        self.v += a_cmd * DT
        self.v = max(MIN_SPEED, min(MAX_SPEED, self.v))

        # Publish
        self.publish_velocity(self.v, d_cmd)

        # Debug
        self.get_logger().info(
            f"Nearest:{self.nearest_ind}/{len(self.waypoints)-1}, " +
            f"dist2goal={dist_goal:.2f}, v={self.v:.2f}, delta={d_cmd:.3f}"
        )

    def publish_velocity(self, linear_x, steering_angle):
        """Publish as Twist for an Ackermann-like vehicle: linear.x = speed, angular.z = steering."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(steering_angle)
        self.cmd_pub.publish(twist)

    def shutdown(self):
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info("Shutting down iterative MPC node.")

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
