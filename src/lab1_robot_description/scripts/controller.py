#!/usr/bin/python3


import rclpy
# from lab1_robot_description.dummy_module import dummy_function, dummy_var
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.create_timer(0.1, self.timmer_callback)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.wheels_pub_velo = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.wheels_pub_position = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)




    def timmer_callback(self):
        pass
    
    def cmd_vel_callback(self,msg:Twist):

        wheelbase = 0.2
        wheel_radius = 0.045

        v = msg.linear.x
        omega = msg.angular.z

        if v != 0:
            steering_angle = math.atan((wheelbase * omega) / v)
        else:
            steering_angle = 0.0

        

        # rotation_deg_front_wheel1 = math.degrees(steering_angle)
        # rotation_deg_front_wheel2 = math.degrees(steering_angle)


        speed_front_wheel1 = v / wheel_radius
        speed_front_wheel2 = v / wheel_radius
        speed_back_wheel1 = v / wheel_radius
        speed_back_wheel2 = v / wheel_radius

        pub_msg = Float64MultiArray()
        pub_msg.data = [
        speed_front_wheel1,
        speed_front_wheel2,
        speed_back_wheel1,
        speed_back_wheel2
        ]

        self.wheels_pub_velo.publish(pub_msg)
        print(steering_angle)
        pub_msg_position = Float64MultiArray()
        pub_msg_position.data = [steering_angle,
        steering_angle]

        self.wheels_pub_position.publish(pub_msg_position)



def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
