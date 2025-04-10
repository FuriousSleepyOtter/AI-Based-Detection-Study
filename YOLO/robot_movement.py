#!/usr/bin/env python3

""" Handles movement """

import rospy
from geometry_msgs.msg import Twist

class RobotMovement:
    def __init__(self):
        # rospy.init_node("robot_movement", anonymous=True)

        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Default velocoty values
        self.linear_speed = 0.2
        self.angular_speed = 0.2

    def move_forward(self):
        """ Moves the robot forward at x speed """
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        """ Stops the robot """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def move_backward(self):
        """ Moves the robot backward """
        twist = Twist()
        twist.linear.x = -self.linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        """ Turns the robot 'left' """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        """ Turns the robot 'right' """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """Keep node running"""
        self.spin()
if __name__ == "__main__":
    robot = RobotMovement()
    robot.run()

