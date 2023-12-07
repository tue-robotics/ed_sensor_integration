import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class table_segmentor:
    def __init__(self) -> None:
        model_path = "~/MEGA/developers/Donal/yolov8n-seg.pt"
        device = "cuda"
        self.model = YOLO(model_path).to(device)
        self.table_class = 60 #table class defined with index 60 ( for laptop testing->person = 0)

        rospy.init_node('listener', anonymous=True)
        self.publisher = rospy.Publisher('/hero/segmented_image',Image,queue_size=None)
        self.subscriber = rospy.Subscriber('/hero/head_rgbd_sensor/rgb/image_raw',Image , self.callback)

    @staticmethod
    def detect(model, frame):
        results = model(frame)
        result = results[0]
        segmentation_contours_idx = [np.array(seg, dtype=np.int32) for seg in result.masks.xy]
        class_ids = np.array(result.boxes.cls.cpu(), dtype="int")
        return class_ids, segmentation_contours_idx


    def callback(self, data):
        rospy.loginfo("got message")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
        rospy.loginfo("converted message")

        classes, segmentations = self.detect(self.model, cv_image)
        #extract table segment and add to frame
        table_mask = np.zeros_like(cv_image, dtype=np.uint8)
        for class_id, seg in zip(classes, segmentations):
            if class_id == self.table_class:
                cv2.polylines(cv_image, [seg], True, (255,0,0), 2)
                cv2.polylines(table_mask, [seg], True, (255,0,0), 2)
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        table_mask_message = bridge.cv2_to_imgmsg(table_mask, encoding="mono8")
        self.publisher.publish(table_mask_message)

    def listener():
        rospy.Subscriber('/hero/head_rgbd_sensor/rgb/image_raw',Image , callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    ts = table_segmentor()
    rospy.spin()
