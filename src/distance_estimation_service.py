#!/usr/bin/env python3
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import rospy
from std_msgs.msg import Bool
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import DistanceEstimation, DistanceEstimationResponse
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

class DistanceEstimationService:
    """
    Service that takes disparity image, stereo image pair and bounding box and
    returns the average 3D point of that
    """
    def __init__(self):
        rospy.loginfo("Object 3D point estimation node is running")
        s = rospy.Service('distance_estimation', DistanceEstimation, self.distance_estimation_handle)
        rospy.spin()

    def distance_estimation_handle(self,req):
        """
        Service Handler - calls function to process the input data,
        extract 3D points from the bounding box and returns the average
        3D point from those
        """
        #rospy.loginfo("Object Estimation Service Started")
        response = DistanceEstimationResponse()
        self.point = Point()
        self.points = []
        self.process_data(req)
        response.object_position = self.get_median_point()
        #print(response.object_position)
        return response

    def process_data(self, data):
        """
        Convert image to numpy array using opencv bridge and
        get camera parameters. Loop through the bounding box
        coordinates to calculate the 3D points
        """
        self.bounding_box = data.bounding_box
        self.disparity_image = data.disparity_image
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
        if disparity == -1 or disparity == 0.0:
            return False
        self.point.z = self.sx/disparity*self.bl;
        self.point.x = (x-self.cx)/self.sx*self.point.z;
        self.point.y = (y-self.cy)/self.sy*self.point.z;
        if self.point.z >= 1000:
            return False
        self.points.append(self.point)

    def get_average_point(self):
        """
        Calculate average 3D point given a vector of 3d PointStamped

        Return point stamped 3D point (with the stamp from the disparity image)
        """
        x = []
        y = []
        z = []
        for p in self.points:
            x.append(p.x)
            y.append(p.y)
            z.append(p.z)
        point = PointStamped()
        point.header = self.disparity_image.header
        point.point.x = np.median(x)
        point.point.y = np.median(y)
        point.point.z = np.median(z)
        return point

def main():
    try:
        rospy.init_node('distance_estimation_service',anonymous=True)
        distance_estimation_service = DistanceEstimationService()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
