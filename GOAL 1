#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import time
import math

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def move_to_goal(self, goal_pose, distance_tolerance):
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # Proportional Controller
            # linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * self.euclidean_distance(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def euclidean_distance(self, goal_pose):
        return math.sqrt(pow((goal_pose.x - self.pose.x), 2) +
                         pow((goal_pose.y - self.pose.y), 2))

if __name__ == '__main__':
    try:
        x = TurtleController()
        goal_pose = Pose()
        # Set your goal position
        goal_pose.x = 1
        goal_pose.y = 1
        x.move_to_goal(goal_pose, distance_tolerance=0.01)
    except rospy.ROSInterruptException:
        pass
