import cv2
from matplotlib import image as img
from ultralytics import YOLO
import numpy as np
import time
def detect(model,img):
    
        results = model.predict(source=img.copy(),save=False,save_txt=False)
        result=results[0]
        segmentation_contours_idx = []
        for seg in result.masks.xy:
            segment = np.array(seg,dtype=np.int32)
            segmentation_contours_idx.append(segment)
        bboxes = np.array(result.boxes.xyxy.cpu(),dtype="int")
        class_ids = np.array(result.boxes.cls.cpu(),dtype="int")
        #scores = np.array(result.boxes.conf.cpu(),dtype="float").round(2)
        return bboxes, class_ids, segmentation_contours_idx#, scores

# def brightness(image,brightness_factor): #positive value -> brighter, negative -> darker
#     # Convert the image to 32-bit float
#     image_float = image.astype(np.float32)
#     # Apply the darkness factor to each pixel
#     darkened_image_float = image_float + brightness_factor
#     # Clip the values to ensure they stay within the valid range [0, 255]
#     darkened_image_float = np.clip(darkened_image_float, 0, 255)
#     # Convert the result back to 8-bit unsigned integer
#     darkened_image = darkened_image_float.astype(np.uint8)
#     return darkened_image

# def change_saturation(image, saturation_factor):
#     # Convert the image from BGR to HSV color space
#     hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#     # Split the HSV image into its components
#     h, s, v = cv2.split(hsv_image)

#     # Apply the saturation factor
#     s = np.clip(s * saturation_factor, 0, 255).astype(np.uint8)

#     # Merge the modified components back into an HSV image
#     modified_hsv = cv2.merge((h, s, v))

#     # Convert the modified HSV image back to BGR color space
#     modified_image = cv2.cvtColor(modified_hsv, cv2.COLOR_HSV2BGR)

#     return modified_image
#Colours dict
colours = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 255),
    'purple': (128, 0, 128),
    # Add more colors as needed
}

#Inputs
#image_path = r"C:\Users\Dónal\Desktop\segment_any\images\corner2.jpg"
model_path = "/home/donal/ros/noetic/system/src/ed_sensor_integration/scripts/yolov8n-seg.pt"
device = "cuda"

#Loads image, converts to BGR colour channel, darkens image and loads model
#image = img.imread(image_path)
#image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
# darkness_factor = -25  # Decrease the brightness by 50 units
# image = brightness(image,darkness_factor) 
# saturation_factor = 0.8
# image = change_saturation(image,saturation_factor)

class_id_to_extract = 0
colours = {
    'yellow': (0, 255, 255),
    'blue': (255, 0, 0),
}

cap = cv2.VideoCapture(0)  # Open the webcam

if not cap.isOpened():
    print("Error: Could not open the camera.")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read a frame.")
        break
    model = YOLO(model_path)
    bboxes, classes, segmentations = detect(model, frame) #, scores = detect(model, frame)

    # Extract the segment of class id 60 (table)
    table_indices = [i for i, class_id in enumerate(classes) if class_id == class_id_to_extract]

    for i in table_indices:
        x, y, x2, y2 = bboxes[i]
        seg = segmentations[i]

        cv2.rectangle(frame, (x, y), (x2, y2), colours['yellow'], 2)
        cv2.polylines(frame, [seg], True, colours['blue'], 2)
        cv2.putText(frame, "Table", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, colours['yellow'], 2)

    cv2.imshow("Video Feed", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
# cap = cv2.VideoCapture(0)

# while cap.isOpened():
#     success, frame = cap.read()

#     if success:
#         start = time.perf_counter()
          
#         model = YOLO(model_path)

#         end = time.perf_counter()
#         total_time = end -start
#         dps = 1/ total_time

#         #extracts data from model
#         bboxes, classes, segmentations, scores = detect(model,frame)

#         #Extracting only table mask
#         for bbox, class_id, seg, score in zip(bboxes, classes, segmentations, scores):
#             (x, y, x2, y2) = bbox
#                 # Table id from trained dataset is 60
#             if class_id == 0:
#                 cv2.rectangle(frame, (x, y), (x2, y2), colours['yellow'], 2) #Colour is Blue Green Red BGR
#                 cv2.polylines(frame, [seg], True, colours['blue'], 2)
#                 # cv2.fillPoly(image, [seg], colours['purple'])
#                 cv2.putText(frame, "Table", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, colours['yellow'], 2)
#         cv2.imshow("Video Feed",frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'): 
#             break
  
# # After the loop release the cap object 
# cap.release() 
# # Destroy all the windows 
# cv2.destroyAllWindows() 




# model = YOLO(model_path)

# #extracts data from model
# bboxes, classes, segmentations, scores = detect(model,image)

# #Extracting only table mask
# for bbox, class_id, seg, score in zip(bboxes, classes, segmentations, scores):
#     (x, y, x2, y2) = bbox
#     # Table id from trained dataset is 60
#     if class_id == 60:
#         cv2.rectangle(image, (x, y), (x2, y2), colours['yellow'], 2) #Colour is Blue Green Red BGR
#         cv2.polylines(image, [seg], True, colours['blue'], 2)
#         # cv2.fillPoly(image, [seg], colours['purple'])
#         cv2.putText(image, "Table", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, colours['yellow'], 2)

# #Visualization
# cv2.imshow('Result Image', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()