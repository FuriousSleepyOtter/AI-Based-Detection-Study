#!/usr/bin/env python3

""" Decides if an obstacle needs to be dodged """

import rospy
import numpy as np
from depth_camera import DepthCamera
from yolo_detection import YOLODetector
from robot_movement import RobotMovement

class ObstacleDetectionFSM:
    def __init__(self):
        # rospy.init_node("obstacle_detection", anonymous=True)
        
        # Initialize modules
        self.depth_camera = DepthCamera()
        self.yolo_detector = YOLODetector()
        self.robot_movement = RobotMovement()

        # Obstacle detection threshold (meters)
        self.safe_distance_obstacles = 1.0
        self.safe_distance_pedestrians = 5.0

        # FSM States
        self.state = "FORWARD"
        self.turn_direction = "clear"

    def check_obstacles(self):

        """ Checks for obstacles and decides movement directions """

        turn_direction_yolo = self.yolo_detector.detect_pedestrians()
        turn_direction_depth, left_obstacles, right_obstacles = self.depth_camera.get_obstacle_direction()

        # Debugging
        # rospy.loginfo(f"YOLO Turn Direction: {turn_direction_yolo}, Depth Turn Direction: {turn_direction_depth}")
        

        # Determine final turn decision
        if turn_direction_yolo != "clear" and abs(left_obstacles - right_obstacles) < 50000:
            self.turn_direction = turn_direction_yolo # Pedestrians has priority
        elif turn_direction_depth != "clear":
            self.turn_direction = turn_direction_depth # Depth camera less priority
        else:
            self.turn_direction = "clear"

        # Debugging
        # rospy.loginfo(f"Actual turn direction: {self.turn_direction}")

    def object_still_in_view(self):

        """ Check if pedestrian is still in the center of the camera view """

        results = self.yolo_detector.model(self.yolo_detector.rgb_image)
        image_width = self.yolo_detector.rgb_image.shape[1]
        center_x_start = image_width // 3
        center_x_end = 2 * image_width // 3

        for result in results:
            for box in result.boxes:
                clas_id = int(box.cls[0])
                confidence = float(box.conf[0])

                if clas_id == 0 and confidence > 0.5:
                    x1, _, x2, _ = map(int, box.xyxy[0])
                    pedestrian_center = (x1 + x2) // 2

                    if center_x_start <= pedestrian_center <= center_x_end:
                        distance = self.yolo_detector.estimate_distance(x2 -x1)

                        if distance is not None and distance < 3.0:
                            rospy.loginfo(f"[FSM] Pedestrian still in danger range")
                            return True
        return False

    def center_fully_clear(self, threshold=3.0):

        """ Check if the center of the image is clear of obstacles. """

        if self.depth_camera.depth_image is None:
            rospy.loginfo("[FSM] No depth image aviable yet.")
            return False

        center_region = self.depth_camera.depth_image[:, self.depth_camera.depth_image.shape[1] // 3: 2 * self.depth_camera.depth_image.shape[1]//3]

        height, width = self.depth_camera.depth_image.shape
        flor_value = 289920 * (width // 3) // (width // 2)
        center_obstacles = max(0, np.sum(center_region < threshold) - flor_value)

        # Debugging 
        # rospy.loginfo(f"[FSM] Center obstacle cout: {center_obstacles}")

        return center_obstacles == 0

    def update_state(self):

        """ FTM to controll robot movement based on detected obstacles """

        self.check_obstacles() # Get lates obstacle info

        rospy.loginfo(f"Current state: {self.state}, Turn direction: {self.turn_direction}")
        

        if self.state == "FORWARD":
            if self.turn_direction != "clear":

                rospy.loginfo(f"Obstacle detected! Switching to TURN_{self.turn_direction.upper()}")

                self.robot_movement.stop()
                rospy.sleep(1)
                self.state = f"TURN_{self.turn_direction.upper()}"
                rospy.loginfo(f"[FSM] State changed to: {self.state}")
                return
            
            else:
                rospy.loginfo("Path is clear, moving forward.")
                self.robot_movement.move_forward()

        elif self.state in ["TURN_LEFT", "TURN_RIGHT"]:
        
            # Keep turning while center its not clear
            while self.object_still_in_view() or not self.center_fully_clear():
                rospy.loginfo("Avoiding obstacle...")
                if self.state == "TURN_RIGHT":

                    # Debugging 
                    rospy.loginfo("[FSM] State set to Turn Right")

                    self.robot_movement.turn_right()
                else:

                    # Debugging 
                    rospy.loginfo("[FSM] State set to Turn Left")
                    
                    self.robot_movement.turn_left()
                rospy.sleep(0.1)

            rospy.loginfo("Path is now clear. Switching to forward state")
            self.robot_movement.move_forward()
            self.state = "FORWARD"
            rospy.loginfo(f"State changed to: {self.state}")

    def run(self):

        """ Continously update the FSM state. """

        rate = rospy.Rate(10) # 10 Hz loop
        while not rospy.is_shutdown():
            self.update_state()
            rate.sleep()

if __name__ == "__main__":
        obstacle_detector_fsm = ObstacleDetectionFSM()
        obstacle_detector_fsm.run()
            