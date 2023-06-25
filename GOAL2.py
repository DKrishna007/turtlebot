#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math



class TurtleBot:
    def _init_(self):
        rospy.init_node('turtlesim_cleaner', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.turtlesim_pose = Pose()
        self.x_min = 0.0
        self.y_min = 0.0
        self.x_max = 11.0
        self.y_max = 11.0
        self.PI = 3.14159265359

    def pose_callback(self, pose_message):
        self.turtlesim_pose.x = pose_message.x
        self.turtlesim_pose.y = pose_message.y
        self.turtlesim_pose.theta = pose_message.theta

    def move(self, speed, distance, is_forward):
        vel_msg = Twist()
        vel_msg.linear.x = abs(speed) if is_forward else -abs(speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        t0 = rospy.Time.now().to_sec()
        current_distance = 0.0
        rate = rospy.Rate(100)
        while current_distance < distance:
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)
            rate.sleep()

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def rotate(self, angular_speed, angle, clockwise):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -abs(angular_speed) if clockwise else abs(angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_angle = 0.0
        rate = rospy.Rate(1000)
        while current_angle < angle:
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def degrees2radians(self, angle_in_degrees):
        return angle_in_degrees * self.PI / 180.0

    def set_desired_orientation(self, desired_angle_radians):
        relative_angle_radians = desired_angle_radians - self.turtlesim_pose.theta
        clockwise = True if relative_angle_radians < 0 else False
        self.rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise)

    def move_goal(self, goal_pose, distance_tolerance):
        vel_msg = Twist()
        rate = rospy.Rate(10)
        while self.get_distance(self.turtlesim_pose.x, self.turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance:
            vel_msg.linear.x = 1.5 * self.get_distance(self.turtlesim_pose.x, self.turtlesim_pose.y, goal_pose.x, goal_pose.y)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (math.atan2(goal_pose.y - self.turtlesim_pose.y, goal_pose.x - self.turtlesim_pose.x) - self.turtlesim_pose.theta)

            self.velocity_publisher.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def grid_clean(self):
        rate = rospy.Rate(0.5)
        goal_pose = Pose()
        goal_pose.x = 1
        goal_pose.y = 1
        goal_pose.theta = 0
        self.move_goal(goal_pose, 0.01)
        rate.sleep()
        self.set_desired_orientation(0)
        rate.sleep()

        self.move(3, 9, True)
        rate.sleep()
        self.rotate(self.degrees2radians(10), self.degrees2radians(90), False)
        rate.sleep()
        self.move(3, 1, True)

        self.rotate(self.degrees2radians(10), self.degrees2radians(90), False)
        rate.sleep()
        self.move(3, 9, True)
        self.rotate(self.degrees2radians(10), self.degrees2radians(270), False)
        rate.sleep()
        self.move(3, 1, True)

        self.rotate(self.degrees2radians(30), self.degrees2radians(90), True)
        rate.sleep()
        self.move(3, 9, True)
        self.rotate(self.degrees2radians(30), self.degrees2radians(270), True)
        rate.sleep()
        self.move(3, 1, True)

if _name_ == "_main_":
    turtle_bot = TurtleBot()
    turtle_bot.grid_clean()
    rospy.spin()
