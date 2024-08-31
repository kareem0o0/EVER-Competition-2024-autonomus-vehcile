#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
import numpy as np
from threading import Thread

#initiate variables 
detected = ''
label_publisher = rospy.Publisher('detected_labels', String, queue_size=10)
# Initialize CvBridge
bridge = CvBridge()

# Set device to GPU if available
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Load the YOLO model and specify the device
#model = YOLO("/home/kareem/catkin_ws/src/t3/scripts/best.pt").to(device)
model = YOLO("/home/kareem/catkin_ws/src/t2/scripts/yolov8n-oiv7.pt").to(device)
model1 = YOLO("/home/kareem/catkin_ws/src/t3/scripts/Cone.pt").to(device)
model2 = YOLO("/home/kareem/catkin_ws/src/t3/scripts/yolov8n.pt").to(device)
def get_centroid(x_min, y_min, x_max, y_max):
    center_x = (x_min + x_max) // 2
    center_y = (y_min + y_max) // 2
    return center_x, center_y

def detect_objects(img):
    global detected 
    detected = "none"
    prev = detected

    # Resize image 
    resized_img = cv2.resize(img, (640, 640))

    # Convert the image to the correct format and device
    img_tensor = torch.from_numpy(resized_img).permute(2, 0, 1).unsqueeze(0).float().to(device)  
    img_tensor /= 255.0  

    results = model.predict(img_tensor)
    results1 = model1.predict(img_tensor)
    results2 = model2.predict(img_tensor)
    for r in results:
        for box in r.boxes:
            label_text = model.names[int(box.cls)]  # Get the class name directly from the model's names
            confidence = box.conf.item()  # Extract the scalar confidence value    
            if label_text in ["Car"]:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                # Format the label to include confidence score
                center_x, center_y = get_centroid(x1, y1, x2, y2)
                label_with_conf = f"{label_text} {confidence:.2f}"
                # Draw the bounding box and label on the resized_img directly
                cv2.rectangle(resized_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.putText(resized_img, label_with_conf, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.circle(resized_img, (center_x, center_y), 5, (0, 255, 0), -1) 
                detected = "car"

   # return resized_img
    for r in results1:
        for box in r.boxes:
            label_text = model1.names[int(box.cls)]  
            confidence = box.conf.item()  
            x1, y1, x2, y2 = map(int, box.xyxy[0])
         
            center_x, center_y = get_centroid(x1, y1, x2, y2)
            label_with_conf = f"cone {confidence:.2f}"
           
            cv2.rectangle(resized_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.putText(resized_img, label_with_conf, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.circle(resized_img, (center_x, center_y), 5, (0, 255, 0), -1) 
            detected = "cone"

    for r in results2:
        for box in r.boxes:
            label_text = model2.names[int(box.cls)]  
            confidence = box.conf.item()  
            if label_text in ["person"]:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
     
                center_x, center_y = get_centroid(x1, y1, x2, y2)
                label_with_conf = f"person {confidence:.2f}"
              
                cv2.rectangle(resized_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.putText(resized_img, label_with_conf, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.circle(resized_img, (center_x, center_y), 5, (0, 255, 0), -1) 
                detected = "person"
    

    label_publisher.publish(detected)
    return resized_img



def image_callback(msg):
    try:
        
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        # Detect objects in the image
        detected_image = detect_objects(cv_image)

        # Display the image with detections
        cv2.imshow("YOLO Object Detection", detected_image)
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr(e)

def main():
    # Initialize ROS node
    rospy.init_node('yolo_image_detection_node')



    # Subscribe to the image topic
    image_subscriber = rospy.Subscriber('image', ROSImage, image_callback)

    # Spin
    rospy.spin()

if __name__ == '__main__':
    main()
