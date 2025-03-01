#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf_transformations import euler_from_quaternion
import yaml


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.create_subscription(Odometry, '/model_odom', self.odom_callback, 10)
        self.gps_publisher = self.create_publisher(PoseStamped, "/gps", 10)
        # self.gps_file = open("gps_data.yaml", "a")

    def add_gaussian_noise(self, data, mean=0, std_dev=0.1):
        return [value + np.random.normal(mean, std_dev) for value in data]
    
    def odom_callback(self, msg:Odometry):
        position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        # velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        # an_vel = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

        pos_n = self.add_gaussian_noise(position)
        ori_n = self.add_gaussian_noise(orientation)
        # vel_n = self.add_gaussian_noise(velocity)
        # an_vel_n = self.add_gaussian_noise(an_vel)

        gps_msg = PoseStamped()
        gps_msg.header.frame_id = "odom"
        gps_msg.header.stamp = self.get_clock().now().to_msg()

        gps_msg.pose.position.x = pos_n[0]
        gps_msg.pose.position.y = pos_n[1]
        gps_msg.pose.position.z = pos_n[2]

        gps_msg.pose.orientation.x = ori_n[0]
        gps_msg.pose.orientation.y = ori_n[1]
        gps_msg.pose.orientation.z = ori_n[2]
        gps_msg.pose.orientation.w = ori_n[3]

        self.gps_publisher.publish(gps_msg)

        # roll, pitch, yaw = euler_from_quaternion([ori_n[0], ori_n[1], ori_n[2], ori_n[3]])
        # # Convert the timestamp to seconds (including nanoseconds).
        # stamp = self.get_clock().now().to_msg()
        # timestamp_sec = stamp.sec + stamp.nanosec * 1e-9

        #  # Create a dictionary to represent the odometry message.
        # data = {
        #     'timestamp': timestamp_sec,
        #     'position': {
        #         'x': pos_n[0],
        #         'y': pos_n[1],
        #         'z': 0.0
        #     },
        #     'orientation': {
        #         'x': 0.0,
        #         'y': 0.0,
        #         'z': yaw
        #     }
        # }

        # # Append the new data as a YAML document.
        # self.gps_file.write("---\n")
        # yaml.dump(data, self.gps_file)
        # self.gps_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
