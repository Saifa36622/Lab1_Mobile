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

        self.wheels_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.wheels_pub_position = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)


    def timmer_callback(self):
        pass
    
    def cmd_vel_callback(self,msg:Twist):
        wheelbase = 0.20
        track_width = 0.14
        wheel_radius = 0.045 

        v = msg.linear.x  
        omega = msg.angular.z

        if omega == 0:
        
            steering_angle_inner = 0.0
            steering_angle_outer = 0.0
        
            wheel_speed = v / wheel_radius
            speed_front_inner = wheel_speed
            speed_front_outer = wheel_speed
            speed_rear_inner = wheel_speed
            speed_rear_outer = wheel_speed
        else:
            R_turn = v / omega
            steering_angle_inner = math.atan(wheelbase / (R_turn - (track_width / 2)))
            steering_angle_outer = math.atan(wheelbase / (R_turn + (track_width / 2)))
        
            # Speeds of each wheel
            speed_front_inner = omega * (R_turn - (track_width / 2)) / wheel_radius
            speed_front_outer = omega * (R_turn + (track_width / 2)) / wheel_radius
            speed_rear_inner = speed_front_inner
            speed_rear_outer = speed_front_outer

        # rotation_deg_front_inner = math.degrees(steering_angle_inner)
        # rotation_deg_front_outer = math.degrees(steering_angle_outer)

        # msg_out = Float64MultiArray()
        # msg_out.data = [rotation_deg_front_inner,rotation_deg_front_outer, speed_front_inner,         
        # speed_front_outer, speed_rear_inner,speed_rear_outer]

        # self.wheels_pub.publish(msg_out)

        print(steering_angle_inner,steering_angle_outer)

        pub_msg = Float64MultiArray()
        pub_msg.data = [
        speed_front_inner,
        speed_front_outer,
        speed_rear_inner,
        speed_rear_outer
        ]

        self.wheels_pub.publish(pub_msg)

        pub_msg_position = Float64MultiArray()
        pub_msg_position.data = [
            steering_angle_inner,
            steering_angle_outer
        ]

        self.wheels_pub_position.publish(pub_msg_position)







def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
