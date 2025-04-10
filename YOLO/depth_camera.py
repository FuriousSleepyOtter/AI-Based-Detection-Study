#!/usr/bin/env python3

""" Handels depth sensing """

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthCamera:
    def __init__(self):
        # rospy.init_node("depth_camera_node", anonymous=True)

        # Subscrive to depth image
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Bridge to convert ROS image to OpenCV
        self.bridge = CvBridge()

        # Store the last depth image
        self.depth_image = None

    def depth_callback(self, msg):
        
        """ Convert ROS depth image to NumPy array. """

        try: 

            # Convert from ROS to CV
            depth_image_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image = np.nan_to_num(depth_image_cv, nan=100, posinf=100, neginf=100)

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")    

    def get_obstacle_direction(self, threshold=3.0, min_obstacle_pixels=5000, floor_value=289920):
        
        """ 
        Analize the depth image and determine best avoidence past
        Results: 'left', 'right', 'clear'
        """

        if self.depth_image is None:
            rospy.loginfo("No depth image received yet.")
            return "clear", 0, 0 # No image yet
    
        # Get image dimensions
        height, width = self.depth_image.shape

        self.depth_image[self.depth_image < 0.1] = 100 # Ignore extremely close noise value
        self.depth_image[self.depth_image > 3.0] = 100 # Ignore extremely far objects

        # Split depth image into 3 regions
        left_half = self.depth_image[:, :width//2]
        right_half = self.depth_image[:, width//2:]
        center_region = self.depth_image[:, width//3:2*width//3] # Middle third of the image

        # Check obstacles in the regions
        center_obstacles = np.sum(center_region < threshold) - (floor_value * (width//3) // (width//2))
        left_obstacles = np.sum(left_half < threshold) - floor_value
        rigth_obstacles = np.sum(right_half < threshold) - floor_value

        # Count pixels with obstacles close
        rospy.loginfo(f"Left: {left_obstacles} Right: {rigth_obstacles} Center: {center_obstacles}")

        if center_obstacles < min_obstacle_pixels:
            return "clear", left_obstacles, rigth_obstacles
        
        turn_direction = "left" if left_obstacles < rigth_obstacles else "right"
        rospy.loginfo(f"Obstacle detected! Returning {turn_direction.upper()} as turn direction.")
        return turn_direction, left_obstacles, rigth_obstacles
        

    def run(self):
        """Keep node running"""
        rospy.spin()

if __name__ == "__main__":
    depth_cam = DepthCamera()
    depth_cam.run()
