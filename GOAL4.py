#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan2


class PoliceTurtle:
    def __init__(self):
        rospy.init_node('police_turtle', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_robber_pose)
        self.pose = Pose()
        self.robber_pose = Pose()
        self.rate = rospy.Rate(10)

    def update_robber_pose(self, data):
        self.robber_pose = data

    def get_distance(self, goal_pose):
        distance = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
        return distance

    def move_to_robber(self):
        goal_pose = Pose()
        velocity_message = Twist()

        while self.get_distance(self.robber_pose) >= 3:
            # Proportional Controller
            # linear velocity in the x-axis:
            velocity_message.linear.x = 1.5 * self.get_distance(self.robber_pose)
            velocity_message.linear.y = 0
            velocity_message.linear.z = 0

            # angular velocity in the z-axis:
            velocity_message.angular.x = 0
            velocity_message.angular.y = 0
            velocity_message.angular.z = 4 * (atan2(self.robber_pose.y - self.pose.y, self.robber_pose.x - self.pose.x) - self.pose.theta)

            self.velocity_publisher.publish(velocity_message)

            self.rate.sleep()

        # Finally, stop the Police Turtle when the distance is below 3 units
        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)


def main():
    try:
        x = PoliceTurtle()
        x.move_to_robber()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
