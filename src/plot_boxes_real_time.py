#!/usr/bin/env python
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
import rospy
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt

class Object_Detection_Test:
    def __init__(self):
        """
        Initialize the box and detected boxes publisher, and define the Subscriber
        for the
        """
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Object Detection Test Started")
        _ = rospy.wait_for_message("camera/left/image_raw", Image)

        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        left_img_sub = message_filters.Subscriber("/scout_1/camera/left/image_raw", Image)
        left_cam_info_sub = message_filters.Subscriber("/scout_1/camera/left/camera_info", CameraInfo)
        right_img_sub = message_filters.Subscriber("/scout_1/camera/right/image_raw", Image)
        right_cam_info_sub = message_filters.Subscriber("/scout_1/camera/right/camera_info", CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([boxes_sub,left_img_sub,left_cam_info_sub,right_img_sub,right_cam_info_sub],10, 0.1, allow_headerless=True)
        ts.registerCallback(self.image_callback)

        self.image_pub = rospy.Publisher("BoundingBox/Image", Image, queue_size = 1)
        rospy.spin()


    def image_callback(self,boxes,left_img,left_cam_info, right_img, right_cam_info):
        """
        Subscriber callback for the stereo camera
        """
        self.boxes = boxes
        self.left_img = left_img
        self.left_cam_info = left_cam_info
        self.right_img = right_img
        self.right_cam_info = right_cam_info
        self.start()

    def start(self):
        """
        transform the ROS img_msg in opencv format, draw bounding boxes and
        writes the label and publishes back as a ROS image_msg to a new topic
        """
        while not rospy.is_shutdown():
            self.bridge = CvBridge()
            original_left_image = self.bridge.imgmsg_to_cv2(self.left_img, "bgr8")
            original_right_image = self.bridge.imgmsg_to_cv2(self.right_img, "bgr8")
            label_list = ["background","cubesat","base_station","base_station_marker",
            "obstacle","volatile","crater","rover"]
            for box in self.boxes.boxes:
                cv2.rectangle(original_left_image,(box.xmin,box.ymin),(box.xmax,box.ymax),[0,255,0],1)
                cv2.putText(original_left_image, label_list[int(box.id)]+": "+str(round(box.confidence, 2)),(box.xmin+1,box.ymax-1),
                fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=0.3, color=(255, 255, 255))
            img_to_publish = self.bridge.cv2_to_imgmsg(original_left_image,"bgr8")
            self.image_pub.publish(img_to_publish)


    def shutdown(self):
        rospy.loginfo("Object Detection Test is shutdown")
        rospy.sleep(1)

def main():
    try:
    	rospy.init_node('object_detection_test', anonymous=True)
        object_detection_test = Object_Detection_Test()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
