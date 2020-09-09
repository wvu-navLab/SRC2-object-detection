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
        rospy.loginfo("Node for converting obstacles to point cloud using disparity image is on")
        self.point_cloud_publisher = rospy.Publisher("inference/point_cloud", PointCloud2, queue_size = 1 )
        self.stereo_subscriber()
        self.seg_points = {}
        self.filtered_points = []
        rospy.sleep(8)
        rospy.spin()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        disparity_sub = message_filters.Subscriber("disparity", DisparityImage)
        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        ts = message_filters.ApproximateTimeSynchronizer([disparity_sub,boxes_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.image_callback)

    def image_callback(self, disparity,boxes):
        """
        Subscriber callback for the disparity images and bounding boxes
        """
        self.disparity = disparity
        self.boxes = boxes
        self.convert_to_point_cloud()

    def convert_to_point_cloud(self):
        """
        Convert to point cloud and publish
        """
        self.points = []
        for box in self.boxes.boxes:
            if box.id == 4 or box.id == 2:
                self.process_data(box)

        # self.cluster_points(thresh = 30)

        scaled_polygon_pcl = PointCloud2()
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(self.boxes.header, self.points)
        self.point_cloud_publisher.publish(scaled_polygon_pcl)

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
        if disparity == -1 or disparity == 0.0:
            return False
        z_ = self.sx/disparity*self.bl;
        x_ = (x-self.cx)/self.sx*z_;
        y_ = (y-self.cy)/self.sy*z_;
        if z_ >= 1000:
            return False
        self.points.append([x_,y_,z_])

    def cluster_points(self, thresh):
        """
        clustering the points using density based clustering method: DBSCAN
        and only keeping the larger clusters

        Input: points, theshold for cluster member size
        ----------------------
        for DBSCAN :

        eps : The maximum distance between two samples for one to be considered as in the neighborhood of the other.
        This is not a maximum bound on the distances of points within a cluster.
        This is the most important DBSCAN parameter to choose appropriately for your data set and distance function.

        min_samples : The number of samples (or total weight) in a neighborhood for a point to be considered as a core point.
        This includes the point itself
        -----------------------
        for KMeans :

        n_clusters : number of clusters
        -----------------------
        thresh : after clustering we remove the points that have size lower than this value
        """
        self.points_np = np.array(self.points)
        clustering = DBSCAN(eps=0.5, min_samples=30).fit(self.points_np)
        # clustering = KMeans(n_clusters=5, random_state=0).fit(self.points)
        labels_np = np.array(clustering.labels_)
        u_labels = np.unique(labels_np)
        for l in u_labels:
            if l != -1:
                self.seg_points[l] = []

        for i in range(len(labels_np)):
            m = clustering.labels_[i]
            if m != -1 :    # -1 label for noisy data
                self.seg_points[m].append(self.points[i])

        for ind, val in self.seg_points.items():
            if len(self.seg_points[ind]) > thresh:
                self.filtered_points.extend(val)

def main():
    try:
        rospy.init_node('object_to_point_cloud',anonymous=True)
        obstacles_to_point_cloud = ObstaclesToPointCloud()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
