#!/usr/bin/env python
"""
Created on Sun Tue 19 22:58:24 2020

@author: Chris Tatsch
"""
import rospy
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
import message_filters #for sincronizing time

class Round3_State_Machine:
    def __init__(self):
        """
        Initialize synchronized image subscriber for the state machine,
        this include pair of stereo camera images, camera info, disparity imgs,
        and the detected bounding boxes
        """
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Object Detection Test Started")

        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        left_img_sub = message_filters.Subscriber("/scout_1/camera/left/image_raw", Image)
        left_cam_info_sub = message_filters.Subscriber("/scout_1/camera/left/camera_info", CameraInfo)
        right_img_sub = message_filters.Subscriber("/scout_1/camera/right/image_raw", Image)
        right_cam_info_sub = message_filters.Subscriber("/scout_1/camera/right/camera_info", CameraInfo)
        disparity_sub = message_filters.Subscriber("DisparityImage", Image)

        ts = message_filters.TimeSynchronizer([boxes_sub,left_img_sub,left_cam_info_sub,right_img_sub,right_cam_info_sub,disparity_img],5)
        ts.registerCallback(self.image_callback)

        self.start()

    def image_callback(self,boxes,left_img,left_cam_info, right_img, right_cam_info,disparity_img):
        """
        Subscriber callback for the stereo camera
        """
        self.boxes = boxes
        self.left_img = left_img
        self.left_cam_info = left_cam_info
        self.right_img = right_img
        self.right_cam_info = right_cam_info
        self.disparity_img = disparity_img


    def start(self):
        """

        """
        while not rospy.is_shutdown():
            #1: Call Server For Drive avoiding obstacles
            #2: Check if base_station or cubesat found:
            #If the first, just call update transformation with respect to the camera frame
            #For the Future have a service that keeps updating that based on Odometry
            #If the second, call the server to stop driving,
            #Call global localization
            #Call position estimation for the cubesat
            #Update and report global position of the cubesat
            #go to drive_to_base


    def check_boxes(self):
        pass


    def call_global_localization(self):
        pass

    def
    def shutdown(self):
        rospy.loginfo("Object Detection Test is shutdown")
        rospy.sleep(1)

def main():
    try:
    	rospy.init_node('round_3 initial state machine', anonymous=True)
        roun3_state_machine = Round3_State_Machine()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
