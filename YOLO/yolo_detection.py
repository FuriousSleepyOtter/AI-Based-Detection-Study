#!/usr/bin/env python3

""" Handles YOLO detection """

import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLODetector:
    def __init__(self):
        rospy.init_node("yolo_detector", anonymous=True)

        # Load YOLO model
        self.model = YOLO("yolov8n.pt") # For YOLO 11 "yolo11n.pt"

        # ROS subscriver
        self.bridege = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/rgb/image_yolo", Image, queue_size=1)
        self.lock = threading.Lock()
       
        # Store images
        self.rgb_image = None

        # Camera parameters
        self.focal_length = 2708 # Depends on each specific camera
        self.real_human_width = 0.46 # Verage human width in meters

        # Image YOLO processing
        self.yolo_thread = threading.Thread(target=self.process_yolo, daemon=True)
        self.yolo_thread.start()
        
    def image_callback(self, msg):

        """ Convert ROS image to OpenCV format. """

        try: 
            self.rgb_image = self.bridege.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")

    def process_yolo(self):

        """ Runs YOLO in a separate thread to avoid blocking the main loop. """

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                if self.rgb_image is not None:
                    rospy.loginfo("[YOLO] Running pedestrian detection...")
                    self.detect_pedestrians()
            rate.sleep()

    def estimate_distance(self, pixel_width):

        """ Estimate the distance to a pedestrian using pinhole camera model. """

        if pixel_width > 0:
            return (self.real_human_width * self.focal_length) / pixel_width
        return None

    def detect_pedestrians(self):

        """ Detects pedestrians and estimate cordinates """

        if self.rgb_image is None:
            return "clear" # No obstacles 
        
        results = self.model(self.rgb_image) # Run YOLO detection
        annotated_image = self.rgb_image.copy()
        image_center_x = self.rgb_image.shape[1] // 2 # Get image with center
        safe_distance_pedestrians = 5.0

        pedestrians = []
        all_pedestrians = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0]) # Get class ID
                confidence = float(box.conf[0]) # Get confidence score

                if class_id == 0 and confidence > 0.5: # Only detecting pedestrians
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    box_width = x2 - x1 # Width of the detected pedestrian in pixels

                    # Estimate distance based on bounding box width
                    distance = self.estimate_distance(box_width)

                   
                    all_pedestrians.append((distance, x1, x2)) # Store distances

                    # Debugging 
                    # rospy.loginfo(f"Pedestrian detected at aprox {distance:.2f}m, Position: {x1}-{x2}")
                    rospy.loginfo(f"[YOLO] Drawing box at ({x1},{y1}) to ({x2},{y2}) for pedestrian at {distance:.2f}m") 
                        
                    # Draw bounding boxses on the image
                    label = f"Person: {distance:.2f}m" if distance is not None else "Unknown"
                    color = (0, 0, 255) if distance is not None and distance < safe_distance_pedestrians else (0, 255, 0)
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated_image, label, (x1, y1 -10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    if distance is None or distance < safe_distance_pedestrians: # Ignor far pedestrians
                        pedestrians.append((distance, x1, x2))


        # Publish image with bounding boxses     
        try:
            self.image_pub.publish(self.bridege.cv2_to_imgmsg(annotated_image, "bgr8"))

            # Debugging 
            rospy.loginfo("[YOLO] Published image with bounding boxses")
        except Exception as e:
            rospy.logerr(f"Error publishing YOLO image: {e}")
        
        if not pedestrians:
            return "clear"
        
        # Get the closes pedestrian
        closest_pedestrian = min(pedestrians, key=lambda p: p[0])
        _, x1, x2 = closest_pedestrian

        # Determine turn direction
        center_x = (x1 + x2) // 2
        turn_direction = "right" if center_x < image_center_x else "left"

        rospy.loginfo(f"Closest pedestrian detecetd. Turning {turn_direction.upper()}")
        return turn_direction
    
    def run(self):
        """Keep the node rinning"""
        rospy.spin()

if __name__ == "__main__":
    yolo = YOLODetector()
    yolo.run()