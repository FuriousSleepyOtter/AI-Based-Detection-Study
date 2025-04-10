#!/usr/bin/env python3

""" To TEST the yolo in the webcam """

import rospy
from ultralytics import YOLO
from cv_bridge import CvBridge  
from sensor_msgs.msg import Image
import cv2

class YOLONode:
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO("yolo11n.pt")

        # Web cam
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            rospy.logerr("Could not oen webcam")
            exit(1)

        # ROS Publisher for the procesed images
        # self.image_pub = rospy.Publisher("/webcam/yolo_output", Image, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            # Frame capture
            ret, frame = self.cap.read()

            if not ret:
                rospy.logerr("Failed to capture frame")
                break

            # Run YOLO
            results = self.model(frame, device="cpu", conf = 0.5)

            # Display results
            for result in results:
                # Get the anotated frames
                annotated_frame = result.plot()

                # Display the frames
                cv2.imshow("YOLO Webcam", annotated_frame)

                # Convert image to ROS mesage
                #result_image = result.plot()
                #ros_image = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
                #self.image_pub.publish(ros_image)

            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release and close
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = YOLONode()
    node.run() 