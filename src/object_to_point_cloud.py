#!/usr/bin/env python
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
from cv_bridge import CvBridge, CvBridgeError
import cv2

import rospy
from std_msgs.msg import Bool
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import message_filters #for sincronizing time

class ObstaclesToPointCloud:
    """
    Convert obstacles bounding boxes to point cloud
    """
    def __init__(self):
        rospy.loginfo("Convert obstacles bounding boxes to point cloud node is on")
        self.point_cloud_publisher = rospy.Publisher("inference/point_cloud", PointCloud2, queue_size = 1 )
        self.stereo_subscriber()
        rospy.sleep(8)
        self.convert_to_point_cloud()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        left_img_sub = message_filters.Subscriber("camera/left/image_raw", Image)
        right_img_sub = message_filters.Subscriber("camera/right/image_raw", Image)
        disparity_sub = message_filters.Subscriber("/disparity", DisparityImage)
        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        ts = message_filters.ApproximateTimeSynchronizer([left_img_sub,right_img_sub,disparity_sub,boxes_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.image_callback)

    def image_callback(self,left_img, right_img, disparity,boxes):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.left_img = left_img
        self.right_img = right_img
        self.disparity = disparity
        self.boxes = boxes


    def convert_to_point_cloud(self):
        """

        """
        while not rospy.is_shutdown():
            self.points = []
            for box in self.boxes.boxes:
                if box.id == 4:
                    self.process_data(box)
            scaled_polygon_pcl = pcl2.create_cloud_xyz32(self.left_img.header, self.points)
            self.point_cloud_publisher.publish(scaled_polygon_pcl)
                    #return response

    def process_data(self, bounding_box):
        """
        Convert image to numpy array using opencv bridge and
        get camera parameters. Loop through the bounding box
        coordinates to calculate the 3D points
        """
        self.bounding_box = bounding_box
        self.disparity_image = self.disparity
        self.bridge = CvBridge()
        self.image = self.bridge.imgmsg_to_cv2(self.disparity_image.image, desired_encoding='passthrough')
        self.cx = float(self.disparity_image.valid_window.width+1)/2
        self.cy = float(self.disparity_image.valid_window.height+1)/2
        self.sx = self.disparity_image.f #Focal length
        self.sy = self.disparity_image.f
        self.bl = self.disparity_image.T # baseline
        for x in range(self.bounding_box.xmin, self.bounding_box.xmax):
            for y in range(self.bounding_box.ymin,self.bounding_box.ymax):
                disparity = self.image.transpose()[x,y] #get x and y on the correct indices
                self.ipt_to_opt(disparity,x,y)

    def ipt_to_opt(self, disparity,x,y):
        """
        Calculate the 3D points x,y,z given 2D point and disparity imageself
        and checking for outliers

        Input: disparity, x 2D point, y 2d point and camera parameters
        """
        disparity = disparity/16
        if disparity == -1:
            return False
        z_ = self.sx/disparity*self.bl;
        x_ = (x-self.cx)/self.sx*z_;
        y_ = (y-self.cy)/self.sy*z_;
        if z_ >= 1000:
            return False
        self.points.append([x_,y_,z_])



def main():
    try:
        rospy.init_node('object_to_point_cloud',anonymous=True)
        obstacles_to_point_cloud = ObstaclesToPointCloud()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
