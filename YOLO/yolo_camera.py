#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from ultralytics import YOLO
import numpy as np

class YOLOCameraNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        # Load YOLO modle
        self.model = YOLO("yolo11n.pt")

    def image_callback(self, data):
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
                    cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)      
            
            # Show procesed image
            cv2.imshow("Robot Camera", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

if __name__ == '__main__':
    YOLOCameraNode()
    rospy.spin()


