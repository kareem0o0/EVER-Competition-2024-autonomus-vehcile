#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from std_msgs.msg import Float32,String

class ConesColorDepth:
    def __init__(self):

        # Initialize YOLO model for cone detection "must be at first"

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO("/home/daino/workspace/src/real_time/scripts/Cone.pt").to(self.device)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.depth_pub = rospy.Publisher('/depth', Float32, queue_size=1)
        self.color_pub = rospy.Publisher('/color', String, queue_size=1)

        self.color_image = None
        self.depth_image = None
        
    def image_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")         #convert ROS Image to bgr Image
            self.color_image = cv2.rotate(self.color_image, cv2.ROTATE_180)    # rotation for real camera
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")       #32-bit Float 1 channel grayscale
            self.depth_image = cv2.rotate(self.depth_image, cv2.ROTATE_180)   # rotation for real camera
        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")

        # Process images if both are available
        if self.color_image is not None and self.depth_image is not None:
            self.process_images()

    def detect_cone_color(self, roi):
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)             #ROI is an input
        
        # Define color ranges for different cone colors
        colors = {
            "red": ((0, 120, 70), (10, 255, 255)),
            "green": ((36, 25, 25), (86, 255, 255)),
            "blue": ((94, 80, 2), (126, 255, 255)),
            "yellow": ((15, 100, 100), (30, 255, 255)),
            "orange": ((5, 50, 50), (15, 255, 255))
        }
        
        max_pixels = 0
        detected_color = "unknown"
        
        for color, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv_roi, lower, upper)
            num_pixels = cv2.countNonZero(mask)
            
            if num_pixels > max_pixels:
                max_pixels = num_pixels
                detected_color = color

        return detected_color

    
    def process_images(self):
        
        if self.color_image is None or self.depth_image is None:
            rospy.logwarn("Color or depth image not available yet.")
            return

        # Resize depth image to match color image dimensions
        depth_resized = cv2.resize(self.depth_image, (self.color_image.shape[1], self.color_image.shape[0]))

        # Convert color image to tensor
        img_tensor = torch.from_numpy(self.color_image).permute(2, 0, 1).unsqueeze(0).float().to(self.device)  
        img_tensor /= 255.0  

        results = self.model.predict(img_tensor)

        closest_cone_depth = float('inf')   #float infinity
        closest_cone_bbox = None
        closest_cone_roi = None

        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                depth_region = depth_resized[y1:y2, x1:x2]
                avg_depth = np.nanmean(depth_region)  # Handle NaN values

                # Update closest cone depth and bounding box
                if avg_depth < closest_cone_depth:
                    closest_cone_depth = avg_depth
                    closest_cone_bbox = (x1, y1, x2, y2)
                    closest_cone_roi = self.color_image[y1:y2, x1:x2]  # Extract region of interest

        # Draw the bounding box for the closest cone
        if closest_cone_bbox is not None:
            x1, y1, x2, y2 = closest_cone_bbox
            cv2.rectangle(self.color_image, (x1, y1), (x2, y2), (255, 0, 255), 2)

            # Detect color of the closest cone
            if closest_cone_roi is not None:
                cone_color = self.detect_cone_color(closest_cone_roi)
                cv2.putText(self.color_image, f"Depth: {closest_cone_depth:.2f} mm", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(self.color_image, f"Cone Color: {cone_color}", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Publish the depth of the closest cone
            self.depth_pub.publish(closest_cone_depth)
            self.color_pub.publish(cone_color)

        # Display the images
        cv2.imshow('Color Image', self.color_image)
        #cv2.imshow('Depth Image', depth_resized)
        cv2.waitKey(1)



def main():
    rospy.init_node('detection_and_distance', anonymous=True)
    ConesColorDepth()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

