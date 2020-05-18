#!/usr/bin/env python

import rospy
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from sensor_msgs.msg import Image
import numpy as np

class Object_Detection_Test:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Object Detection Test Started")
        self.box_pub = rospy.Publisher("Box", Box,queue_size=1)
        self.boxes_pub = rospy.Publisher("DetectedBoxes", DetectedBoxes, queue_size = 1)
        self. left_img = Image()
        rospy.Subscriber("/scout_1/camera/left/image_raw", Image, self.left_image_callback)


        self.start()

    def left_image_callback(self,data):
        self.left_img = data


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
            boxes.header = self.left_img.header
            boxes.boxes = [box,box,box,box]
            self.boxes_pub.publish(boxes)
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
