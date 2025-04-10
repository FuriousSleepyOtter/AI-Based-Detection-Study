#!/usr/bin/env python3

""" Main code """

import rospy
from obstacle_detection import ObstacleDetectionFSM
from robot_movement import RobotMovement

class RobotController:
    def __init__(self):
        rospy.init_node("robot_controller", anonymous=True)

        # Initialize modules
        self.obstacle_detection = ObstacleDetectionFSM()
        self.robot_movement = RobotMovement()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):

        """ Stop robot when script is terminated """

        rospy.loginfo("Shuting down... Stopping robot.")
        self.robot_movement.stop()

    def run(self):

        """ Main loop to continiously check obstacles and control the robot """

        rospy.loginfo("Robot started.")
        self.obstacle_detection.run() # Runs obstacle detection loop

if __name__ == "__main__":
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass # Handle shutdown