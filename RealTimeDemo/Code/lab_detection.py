import cv2
import torch
from ultralytics import YOLO
import numpy as np

# Set device to GPU if available
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Load the YOLO model and specify the device
model = YOLO("/home/daino/workspace/src/milestone3/scripts/yolov8n-oiv7.pt").to(device)
model1 = YOLO("/home/daino/workspace/src/milestone3/scripts/Cone.pt").to(device)
model2 = YOLO("/home/daino/workspace/src/milestone3/scripts/yolov8n.pt").to(device)

def get_centroid(x_min, y_min, x_max, y_max):
    center_x = (x_min + x_max) // 2
    center_y = (y_min + y_max) // 2
    return center_x, center_y

def detect_cone_color(roi):
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
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

def detect_objects(img):
    global detected 
    detected = "none"

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
                label_with_conf = f'{label_text} {confidence:.2f}'
                # Draw the bounding box and label on the resized_img directly
                cv2.rectangle(resized_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.putText(resized_img, label_with_conf, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.circle(resized_img, (center_x, center_y), 5, (0, 255, 0), -1) 
                detected = "car"

    for r in results1:
        for box in r.boxes:
            label_text = model1.names[int(box.cls)]  
            confidence = box.conf.item()  
            x1, y1, x2, y2 = map(int, box.xyxy[0])
         
            center_x, center_y = get_centroid(x1, y1, x2, y2)
            roi = resized_img[y1:y2, x1:x2]
            cone_color = detect_cone_color(roi)
            label_with_conf = f"{cone_color} cone {confidence:.2f}"
           
            cv2.rectangle(resized_img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.putText(resized_img, label_with_conf, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.circle(resized_img, (center_x, center_y), 5, (0, 255, 0), -1) 
            detected = f"{cone_color} cone"

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
    
    return resized_img

def main():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        detected_image = detect_objects(frame)
        cv2.imshow("YOLO Object Detection", detected_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()