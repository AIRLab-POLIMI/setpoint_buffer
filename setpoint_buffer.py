#!/usr/bin/python
# -*- coding: utf-8 -*-
import rclpy 
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# from offboard.attitude_library import Attitude
from attitude_library import Attitude
# from offboard.guidance_library import *
from guidance_library import *

import time
import math
import numpy as np

class SetpointBuffer(Node):

    ''' Callbacks'''

    def local_pose_callback(self, msg):
        print('first callback')
        self.local_pose = msg

    def set_target_pose_callback(self, msg):
        print('second callback')
        self.cur_target_pose = msg

    def __init__(self):
        super().__init__('setpoint_buffer_node')

        self.attitude = Attitude()
        self.mode = FlightModes()
        self.controller = Controller()

        self.local_pose = PoseStamped()
        self.takeoff_flag = String()
        self.takeoff_flag.data = "True"

        self.arm_state = False
        self.offboard_state = False

        self.takeoff_height = self.controller.takeoff_height
        self.takeoff_speed = self.controller.takeoff_speed
        self.cur_target_pose = None

        rate = 20
        self.rate_of_spin = 1/rate
        self.rate = self.create_rate(rate)

        '''
        ROS Subscribers
        '''
        # self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        # self.set_target_position_sub = rospy.Subscriber("commander/set_pose", PositionTarget, self.set_target_pose_callback)
        self.local_pose_sub = self.create_subscription(PoseStamped, '/rogx_vision_mockup/mavros/local_position/pose', self.local_pose_callback, 10)
        self.set_target_position_sub = self.create_subscription(PositionTarget, '/rogx_vision_mockup/commander/set_pose', self.set_target_pose_callback, 10)
        '''
        ROS Publishers
        '''
        self.local_target_pub = self.create_publisher(PositionTarget, '/rogx_vision_mockup/mavros/setpoint_raw/local', 10)
        # self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        print("Px4 Controller Initialized!")
        time.sleep(2)
        self.start() # starting the start!

    def rad2deg(self, angle_rad):

        angle_deg = angle_rad / np.pi * 180.0

        return angle_deg

    def start(self):

        x_in = self.local_pose.pose.position.x
        y_in = self.local_pose.pose.position.y
        z_in = self.local_pose.pose.position.z
        yaw_in_rad = self.controller.current_yaw(self.local_pose)
        yaw_in_deg = self.rad2deg( yaw_in_rad )

        self.cur_target_pose = self.controller.construct_target(x_in, y_in, self.takeoff_height, yaw_in_rad)

        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose)
            self.offboard_state = self.mode.offboard()
            print('inside the for')
            # self.rate.sleep() #some issues!
            rclpy.spin_once(self, timeout_sec=self.rate_of_spin)
            print('after the for')

        self.arm_state = self.mode.arm()
        print('this step 1')
        distance = self.takeoff_height
        print('this step 2')
        occurred_time = distance / self.takeoff_speed
        print('this step 3')
        ti = time.time()
        tf = ti + occurred_time

        print('this step 4')
        while time.time() < tf:
            x, y, z = self.controller.linear_trajectory(x_in, y_in, z_in, x_in, y_in, self.takeoff_height, ti, tf)
            target_controller = self.controller.move_and_yaw(x, y, z, yaw_in_deg, self.local_pose)
            self.local_target_pub.publish(target_controller)
            rclpy.spin_once(self, timeout_sec=self.rate_of_spin)
            # self.rate.sleep()

        # while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
        print('this step 5')
        while rclpy.ok() and self.arm_state and self.offboard_state:
            self.local_target_pub.publish(self.cur_target_pose)
            print('published')
            # self.rate.sleep()
            rclpy.spin_once(self, timeout_sec=self.rate_of_spin)

def main(args=None):
    rclpy.init(args=args)
    node = SetpointBuffer()
    rclpy.spin(node)
    # node.start()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
