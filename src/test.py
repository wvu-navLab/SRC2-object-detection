#!/usr/bin/env python

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
        self.box_pub = rospy.Publisher("Box", Box,queue_size=1)
        self.boxes_pub = rospy.Publisher("DetectedBoxes", DetectedBoxes, queue_size = 1)

        self.left_img = Image()

        left_img_sub = message_filters.Subscriber("/scout_1/camera/left/image_raw", Image)
        left_cam_info_sub = message_filters.Subscriber("/scout_1/camera/left/camera_info", CameraInfo)
        right_img_sub = message_filters.Subscriber("/scout_1/camera/right/image_raw", Image)
        right_cam_info_sub = message_filters.Subscriber("/scout_1/camera/right/camera_info", CameraInfo)
        ts = message_filters.TimeSynchronizer([left_img_sub,left_cam_info_sub,right_img_sub,right_cam_info_sub],5)
        ts.registerCallback(self.image_callback)

        rospy.sleep(5)
        self.start()

    def image_callback(self,left_img,left_cam_info, right_img, right_cam_info):
        """
        Subscriber callback for the stereo camera
        """
        self.left_img = left_img
        self.left_cam_info = left_cam_info
        self.right_img = right_img
        self.right_cam_info = right_cam_info


    def start(self):
        while not rospy.is_shutdown():
            box = Box()
            box.id = 1
            box.xmin = np.random.randint(0,500)
            box.ymin = np.random.randint(0,500)
            box.xmax = np.random.randint(0,500)
            box.ymax = np.random.randint(0,500)
            self.box_pub.publish(box)
            #print(self.left_img.header)
            boxes = DetectedBoxes()
            #boxes.header = self.left_img.header
            boxes.boxes = [box,box,box,box]
            self.boxes_pub.publish(boxes)
            #print(self.left_img.header, self.right_img.header )
            self.bridge = CvBridge()
            original_image = self.bridge.imgmsg_to_cv2(self.left_img, "bgr8")
            resize_image = cv2.resize(original_image, dsize=(300, 300), interpolation=cv2.INTER_CUBIC)
            print(resize_image.shape)
            plt.imshow(resize_image)
            plt.show()

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
