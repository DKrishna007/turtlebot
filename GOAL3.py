#!/usr/bin/env python3

import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64

class TurtleBot:
    def _init_(self):
        rospy.init_node('turtlesim_circular', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_publisher = rospy.Publisher('/rt_real_pose', Pose, queue_size=10)
        self.noisy_pose_publisher = rospy.Publisher('/rt_noisy_pose', Pose, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.turtlesim_pose = Pose()
        self.rate = rospy.Rate(0.2)  
        self.noise_std_dev = 10.0

    def pose_callback(self, pose_message):
        self.turtlesim_pose = pose_message
        self.pose_publisher.publish(self.turtlesim_pose)
        noisy_pose = Pose()
        noisy_pose.x = pose_message.x + random.gauss(0, self.noise_std_dev)
        noisy_pose.y = pose_message.y + random.gauss(0, self.noise_std_dev)
        noisy_pose.theta = pose_message.theta
        self.noisy_pose_publisher.publish(noisy_pose)

    def move_in_circle(self, linear_speed, angular_speed):
        vel_msg = Twist()
        while not rospy.is_shutdown():
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

if _name_ == "_main_":
    turtle_bot = TurtleBot()
    linear_speed = 1.0  
    angular_speed = 0.5 
    turtle_bot.move_in_circle(linear_speed, angular_speed)
