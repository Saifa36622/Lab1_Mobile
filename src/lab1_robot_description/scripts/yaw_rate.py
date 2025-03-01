#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import yaml

class YawRateNode(Node):
    def __init__(self):
        super().__init__('yaw_rate_node')

        self.dt = 0.01

        self.declare_parameter('pose', [0.0, 0.0, 0.0])
        self.start_pose = self.get_parameter('pose').get_parameter_value().double_array_value 

        self.create_timer(self.dt, self.timer_callback)
        
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.odom_publisher1 = self.create_publisher(Odometry, "/yaw_rate/odom", 10)

        self.odom = [self.start_pose[0],self.start_pose[1],self.start_pose[2],0,0,0] #x, y, theta, beta, v, w
        self.vel_rear = [0,0] #right, left
        self.w = 0.0
        self.wheel_radius = 0.045

        # self.odom_file = open("yaw_rate.yaml", "a")

    def timer_callback(self):
        new_odom = [0,0,0,0,0,0]
        new_odom[0] = self.odom[0] + (self.odom[4] * self.dt * np.cos(self.odom[3] + self.odom[2] + (self.odom[5]*self.dt/2)))
        new_odom[1] = self.odom[1] + (self.odom[4] * self.dt * np.sin(self.odom[3] + self.odom[2] + (self.odom[5]*self.dt/2)))
        new_odom[2] = self.odom[2] + (self.odom[5]*self.dt)
        new_odom[4] = (self.vel_rear[0] + self.vel_rear[1])/2
        new_odom[5] = self.w
        self.odom = new_odom
        self.odom_pub(self.odom)

    def imu_callback(self, msg:Imu):
        self.w = msg.angular_velocity.z

    def joint_states_callback(self, msg:JointState):
        index_l, index_r = None, None
        for i in range(len(msg.name)):
            if msg.name[i] == "left_joint_b":
                index_l = i
            elif msg.name[i] == "right_joint_b":
                index_r = i

        if index_l is not None and index_r is not None:
            self.vel_rear = [msg.velocity[index_r]*self.wheel_radius, msg.velocity[index_l]*self.wheel_radius]

    def odom_pub(self, odom):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "yaw_rate"
        odom_msg.pose.pose.position.x = odom[0]
        odom_msg.pose.pose.position.y = odom[1]

        q = quaternion_from_euler(0 ,0 , odom[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = odom[4]
        odom_msg.twist.twist.angular.z = odom[5]
        self.odom_publisher1.publish(odom_msg)

        # # Convert the timestamp to seconds (including nanoseconds).
        # stamp = odom_msg.header.stamp
        # timestamp_sec = stamp.sec + stamp.nanosec * 1e-9

        # # Extract pose and twist information.
        # pos = odom_msg.pose.pose.position
        # ori = odom_msg.pose.pose.orientation
        # lin = odom_msg.twist.twist.linear
        # ang = odom_msg.twist.twist.angular
        # roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z,ori.w])

        #  # Create a dictionary to represent the odometry message.
        # data = {
        #     'timestamp': timestamp_sec,
        #     'position': {
        #         'x': float(pos.x),
        #         'y': float(pos.y),
        #         'z': float(pos.z)
        #     },
        #     'orientation': {
        #         'x': float(roll),
        #         'y': float(pitch),
        #         'z': float(yaw)
        #     },
        #     'linear': {
        #         'x': float(lin.x),
        #         'y': float(lin.y),
        #         'z': float(lin.z)
        #     },
        #     'angular': {
        #         'x': float(ang.x),
        #         'y': float(ang.y),
        #         'z': float(ang.z)
        #     }
        # }

        # # Append the new data as a YAML document.
        # self.odom_file.write("---\n")
        # yaml.dump(data, self.odom_file)
        # self.odom_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = YawRateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
