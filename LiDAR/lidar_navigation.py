#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ReactiveNavigation:
    def __init__(self):

        """ Initialize ROS node and setup LIDAR-based obstacle avoidance. """

        rospy.init_node("reactive_navigation", anonymous=True)

        # Subscribers
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Publisher 
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Default movement parameters
        self.safe_distance = 0.8 # Minimum distance before turning
        self.turn_speed = 0.6     # Angular speed 
        self.forward_speed = 0.5  # Linear speed 

        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("Reactive Navigation Node Initialized.")

    def lidar_callback(self, data):

        """ Process LiDAR scan data and adjust movement based on obstacles. """

        max_range = 3.5 # Lidar max range in m
        ranges = [r if not math.isinf(r) else max_range for r in data.ranges]
        twist = Twist()

        #Debugging
        # rospy.loginfo(f"Raw LiDAR data First 10: {data.ranges[:360]}")

        # Front reading at 0º
        self.front_distance = min(ranges[:20] + ranges[-20:])

        # Left and right
        self.left_distance = min(ranges[75:105])  # Leftmost 30 degrees
        self.right_distance = min(ranges[255:285])  # Rightmost 30 degrees

        rospy.loginfo(f"Front: {self.front_distance:.2f}m | Left: {self.left_distance:.2f}m | Right:{self.right_distance:.2f}m")
        
        if self.front_distance < self.safe_distance:
            # If an obstacle is in front turn
            if self.left_distance > self.right_distance:
                rospy.loginfo("Obstacle ahead! Turning left.")
                twist.angular.z = self.turn_speed
            else:
                rospy.loginfo("Obstacle ahead! Turning right.")
                twist.angular.z = -self.turn_speed
        else:
            # Move forward if path is clear
            twist.linear.x = self.forward_speed
            rospy.loginfo("Moving forward.")

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):

        """ Stops the robot when shoutdown. """

        rospy.loginfo("Stopping Robot...")
        stop_cmd = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(stop_cmd)
            rospy.sleep() 

if __name__ == "__main__":
    try:
        node = ReactiveNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node Interrupted!")