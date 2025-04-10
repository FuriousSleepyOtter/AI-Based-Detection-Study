#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import sys
import termios
import tty
import select
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from ultralytics import YOLO

class YOLOCameraNode:
    def __init__(self):
        rospy.init_node("yolo_camera_node", anonymous=True)

        self.bridge = CvBridge()  
        twist = Twist()     

        # Load YOLO modle
        self.model = None
        try:
            self.model = YOLO("yolov8n.pt")
        except Exception as e:
            rospy.signal_shutdown("YOLO Model Load Failure")

        if self.model:
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
            self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
            return

    def image_callback(self, data):
        
        """ Proces the camera image and runs YOLO detection. """

        if self.model is None:
            rospy.logwarn("[YOLO] Skipping image processing, model not loaded yet.")
            return
        
        try:
            # COnvert image from ROS to CV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # YOLO detection
            results = self.model(cv_image)
            
            # Draw the detection on the image
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0]) # Bounding boxes
                    conf = float(box.conf[0]) # confidence score
                    cls = int(box.cls[0]) # class indes
                    label = f"{self.model.names[cls]}: {conf:.2f}"

                    # Draw rectangle and label
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (x1, y2 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)      
            
            # Show procesed image
            cv2.imshow("Robot Camera", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def get_key(self):
        
        """ Reads a single key press without blocking. """

        settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def stop_robot(self):

        """ Stop the robot before exiting. """

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Robot Stoped")
    
    def control_loop(self):

        """ Handles robot movement based on key imputs. """

        twist = Twist()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            key = self.get_key()

            if key in ["w", "\x1b[A"]:
                twist.linear.x = 0.0
                twist.angular.x = 0.0
                twist.linear.x = 0.7
            elif key in ["s", "\x1b[B"]:
                twist.linear.x = 0.0
                twist.angular.x = 0.0
                twist.linear.x = -0.7
            elif key in ["a", "\x1b[D"]:
                twist.linear.x = 0.0
                twist.angular.x = 0.0
                twist.angular.z = 0.5
            elif key in ["d", "\x1b[C"]:
                twist.linear.x = 0.0
                twist.angular.x = 0.0
                twist.angular.z = -0.5
            elif key in ["space"]:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == None:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key in ["p"]:
                break
            elif key == "\x03":
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0


            rospy.loginfo(f"[CMD] Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    node = YOLOCameraNode()
    try:
        node.control_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
    


