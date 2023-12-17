import cv2
from ultralytics import YOLO
import numpy as np
import time

def detect(model, frame):
    results = model(frame)
    result = results[0]
    segmentation_contours_idx = [np.array(seg, dtype=np.int32) for seg in result.masks.xy]
    class_ids = np.array(result.boxes.cls.cpu(), dtype="int")
    return class_ids, segmentation_contours_idx

colours = {
    'yellow': (0, 255, 255),
    'blue': (255, 0, 0),
}

model_path = "/home/donal/ros/noetic/system/src/ed_sensor_integration/scripts/yolov8n-seg.pt"
device = "cpu"
model = YOLO(model_path).to(device)
table_class = 0 #table class defined with index 60 (person = 0)

# Detection Loop with webcam
cap = cv2.VideoCapture(0)  

if not cap.isOpened():
    print("Error: Could not open the camera.")
    exit()

# Initialize refresh rate calc
start_time = time.time()
frame_count = 0

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read a frame.")
        break
    #Get classes and segments
    classes, segmentations = detect(model, frame)
    #extract table segment and add to frame
    for class_id, seg in zip(classes, segmentations):
        if class_id == table_class: 
            cv2.polylines(frame, [seg], True, colours['blue'], 2)
    # Calculate the refresh rate for segmentation
    frame_count += 1
    end_time = time.time()
    elapsed_time = end_time - start_time
    segmentation_frame_rate = int(frame_count / elapsed_time)
    start_time = end_time
    frame_count = 0
    # Display segmentation refresh rate
    cv2.putText(frame, f"Seg FPS: {segmentation_frame_rate}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    cv2.imshow("Video Feed", frame)
    #press q to quit window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()