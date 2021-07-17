#!/usr/bin/env python3

"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
# ROS Libraries
import rospy
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import message_filters #for sincronizing time

from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
from scipy import ndimage
import numpy as np
import rospkg


# System Libraries
import glob
import os


print_to_terminal = rospy.get_param('obstacle_detection/print_to_terminal', False)


class Obstacle_Detection:
    def __init__(self):
        """
        Initialize the obstacle point cloud publisher, define subscriber for the image
        topics
        """
        rospy.on_shutdown(self.shutdown)
        self.point_cloud_publisher = rospy.Publisher("obstacle/point_cloud", PointCloud2, queue_size = 1 )
        rospy.loginfo("Object Detection Inference Started")
        self.stereo_subscriber()
        self.start()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        left_img_sub = message_filters.Subscriber("/small_scout_1/camera/left/image_raw", Image)

        # print(left_img_sub)
        #left_cam_info_sub = message_filters.Subscriber("camera/left/camera_info", CameraInfo)
        right_img_sub = message_filters.Subscriber("/small_scout_1/camera/right/image_raw", Image)
        disparity_sub = message_filters.Subscriber("/small_scout_1/disparity", DisparityImage)

        #right_cam_info_sub = message_filters.Subscriber("camera/right/camera_info", CameraInfo)
        #ts = message_filters.ApproximateTimeSynchronizer([left_img_sub,left_cam_info_sub,right_img_sub,right_cam_info_sub],10, 0.1, allow_headerless=True)
        ts = message_filters.ApproximateTimeSynchronizer([left_img_sub,right_img_sub,disparity_sub],10, 0.1, allow_headerless=True)
        ts.registerCallback(self.image_callback)

    #def image_callback(self,left_img,left_cam_info, right_img, right_cam_info):
    def image_callback(self, left_img, right_img, disparity):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.left_img = left_img
        self.disparity = disparity
        #self.left_cam_info = left_cam_info
        #self.right_img = right_img
        #self.right_cam_info = right_cam_info

    def start(self):
        """
            Loop through transforming the subscribed img_msg to
            an array, resizing it to the 300x300 network input formatself.
            Then running the inference on the network model and publishing
            the bounding boxes as a custom msg
        """
        rospy.sleep(3)
        while not rospy.is_shutdown():
            self.bridge = CvBridge()
            original_image = self.bridge.imgmsg_to_cv2(self.left_img, "bgr8")
            gray_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)
            #maybe add gaussian blur
            original_disparity_image = self.bridge.imgmsg_to_cv2(self.disparity.image, desired_encoding ="passthrough")
            edges = cv2.Canny(gray_image,50,200)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 200, minLineLength=10, maxLineGap=250)
            cv2.imshow('image',original_image)
            cv2.imshow('image2',original_disparity_image)
            cv2.imshow('image3',edges)
            print(lines)
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(original_image,(x1,y1), (x2,y2), (0,0,255),2)
            cv2.imshow('image4',original_image)

            cv2.waitKey(0)





    def shutdown(self):
        """
        Shutdown Node
        """
        rospy.loginfo("Object Detection Inference is shutdown")
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('obstacle_detection_inference', anonymous=True)
        object_detection_test = Obstacle_Detection()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
