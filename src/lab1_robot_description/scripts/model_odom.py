#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point, Vector3, Quaternion
from tf_transformations import euler_from_quaternion
import numpy as np
import yaml

class ModelNode(Node):
    def __init__(self):
        super().__init__('model_node')

        self.dt = 0.01
        self.create_timer(self.dt, self.timer_callback)

        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        self.odom_publisher = self.create_publisher(Odometry, "/model_odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # self.odom_file = open("odom_data.yaml", "a")
        self.pos = Point()
        self.ori = Quaternion()
        self.lin = Vector3()
        self.ang = Vector3()

    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.pos.x
        t.transform.translation.y = self.pos.y
        t.transform.translation.z = self.pos.z
        t.transform.rotation = self.ori
        # ส่ง TF
        self.tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position = self.pos
        odom_msg.pose.pose.orientation = self.ori
        odom_msg.twist.twist.linear = self.lin
        odom_msg.twist.twist.angular = self.ang
        self.odom_publisher.publish(odom_msg)
        print("pub")

        # # Convert the timestamp to seconds (including nanoseconds).
        # stamp = self.get_clock().now().to_msg()
        # timestamp_sec = stamp.sec + stamp.nanosec * 1e-9
        # roll, pitch, yaw = euler_from_quaternion([self.ori.x, self.ori.y, self.ori.z, self.ori.w])

        #  # Create a dictionary to represent the odometry message.
        # data = {
        #     'timestamp': timestamp_sec,
        #     'position': {
        #         'x': self.pos.x,
        #         'y': self.pos.y,
        #         'z': self.pos.z
        #     },
        #     'orientation': {
        #         'x': float(roll),
        #         'y': float(pitch),
        #         'z': float(yaw)
        #     },
        #     'linear': {
        #         'x': float(np.sqrt(self.lin.x**2 + self.lin.y**2)),
        #         'y': 0.0,
        #         'z': self.lin.z
        #     },
        #     'angular': {
        #         'x': self.ang.x,
        #         'y': self.ang.y,
        #         'z': self.ang.z
        #     }
        # }

        # # Append the new data as a YAML document.
        # self.odom_file.write("---\n")
        # yaml.dump(data, self.odom_file)
        # self.odom_file.flush()
        

    def model_states_callback(self, msg:ModelStates):
        for i in range(len(msg.name)):
            if msg.name[i] == "example":
                index = i

        # Extract pose and twist information.
        self.pos = msg.pose[index].position
        self.ori = msg.pose[index].orientation
        self.lin = msg.twist[index].linear
        self.ang = msg.twist[index].angular

def main(args=None):
    rclpy.init(args=args)
    node = ModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
