#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class table_segmentor:
    def __init__(self) -> None:
        model_path = "~/MEGA/developers/Donal/yolov8n-seg.pt"
        device = "cpu"
        self.model = YOLO(model_path).to(device)
        self.table_class = 60 #table class defined with index 60 (person = 0)

        rospy.init_node('listener', anonymous=True)
        self.publisher = rospy.Publisher('/hero/segmented_image',Image,queue_size=10)
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
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        rospy.loginfo("converted message")

        classes, segmentations = self.detect(self.model, cv_image)
        #extract table segment and add to frame
        for class_id, seg in zip(classes, segmentations):
            if class_id == self.table_class:
                cv2.polylines(cv_image, [seg], True, (255,0,0), 2)
        # cv2.imshow("Segmented Image", cv_image)
        # cv2.waitKey(1)
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        self.publisher.publish(image_message)

    def listener():

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.


        rospy.Subscriber('/hero/head_rgbd_sensor/rgb/image_raw',Image , callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    ts = table_segmentor()
    rospy.spin()
