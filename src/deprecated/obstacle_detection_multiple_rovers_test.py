#!/usr/bin/env python3

## DEPRECATED ##

"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
from cv_bridge import CvBridge, CvBridgeError
import cv2

import open3d as o3d

import numpy as np
import rospy
from std_msgs.msg import Bool
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import FindObject, FindObjectResponse, FindObjectRequest
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import message_filters #for sincronizing time
import rospkg

# list_of_robots = rospy.get_param('robots_list', ["small_scout_1", "small_scout_2","small_hauler_1","small_hauler_2","small_excavator_1","small_excavator_2"]) #List of robots that are being used
list_of_robots = rospy.get_param('robots_list', ["small_scout_1","small_hauler_1","small_excavator_1"]) #List of robots that are being used
#list_of_robots = rospy.get_param('robots_list', ["small_scout_1", "small_scout_2", "small_excavator_1"]) #List of robots that are being used


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


class ObstaclesToPointCloudMultipleRovers:
    """
    Convert obstacles bounding boxes to point cloud
    """
    def __init__(self):
        rospy.loginfo("Node for converting obstacles to point cloud using disparity image is on")
        rospy.on_shutdown(self.shutdown)
        self.rgb_images = {key: None for key in list_of_robots}
        self.disparity_images = {key: None for key in list_of_robots}
        self.point_cloud_publishers = {key: rospy.Publisher(key+"/inference/point_cloud",
                                        PointCloud2, queue_size = 1 ) for key in list_of_robots}
        self.publisher = rospy.Publisher("Dummy_plublisher", String, queue_size =1)

        self.images_subscriber()

#        self.stereo_subscriber()
#        self.seg_points = {}
#        self.filtered_points = []
        rospy.sleep(1)
        self.detect_obstacles()



    def images_subscriber(self):
        """
        Define the Subscriber with for multiple robots left image and disparity topics
        (later might need to change back to message filter to improve performance)
        """
        for robot in list_of_robots:
            rospy.Subscriber(robot+"/camera/left/image_raw", Image, self.image_callback, robot)
            rospy.Subscriber(robot+"/disparity", DisparityImage, self.disparity_callback, robot)

        #disparity_sub = message_filters.Subscriber("disparity", DisparityImage)
        #boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        #ts = message_filters.ApproximateTimeSynchronizer([disparity_sub,boxes_sub], 10, 0.1, allow_headerless=True)
        #ts.registerCallback(self.image_callback)

    def image_callback(self, img,robot):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.rgb_images[robot] = img

    def disparity_callback(self, disparity_img,robot):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.disparity_images[robot] = disparity_img

    def detect_obstacles(self):
        """
        Loop to run object detection and convert to point cloud for all the robot
        """
        rate = rospy.Rate(0.2) # ROS Rate at 5Hz
        watch_dog_timer = 0
        robot_boxes = DetectedBoxes()

        while not rospy.is_shutdown():

            #DO OPENCV STUFF HERE
            #####################
            #####################
            #####################

            # watch_dog_timer +=1
            # if watch_dog_timer >5: # run inference at slower rate
            #     watch_dog_timer = 0
            #     for robot in list_of_robots:
            #         rospy.wait_for_service('/find_object')
            #         _find_object =rospy.ServiceProxy('/find_object', FindObject)
            #         try:
            #             _find_object = _find_object(robot_name = robot)
            #         except rospy.ServiceException as exc:
            #             print("Service did not process request: " + str(exc))
            #         robot_boxes = _find_object.boxes
            #         self.convert_box_to_point_cloud(robot_boxes,robot)
            # self.publisher.publish(String("Hi"))



            #rospy.wait_for_service('/find_object')
            #_find_object =rospy.ServiceProxy('/find_object', FindObject)
            #try:
            #    _find_object = _find_object(robot_name = "small_excavator_1")
            #except rospy.ServiceException as exc:
            #    print("Service did not process request: " + str(exc))
            #robot_boxes = _find_object.boxes

            self.convert_box_to_point_cloud(robot_boxes,"small_excavator_1")
            rate.sleep()

    def convert_box_to_point_cloud(self, robot_boxes, robot_name):
        """
        Convert to point cloud and publish
        """
        self.points = []
        self.color = []
        #for box in robot_boxes.boxes:
        #    if (box.id == 0 or box.id == 1 or box.id == 2 or
        #    box.id == 3 or box.id == 4 or box.id == 5 or box.id == 6):
        box = Box()
        box.xmin = 0
        box.ymin = 0
        box.xmax = 640
        box.ymax = 480
        self.process_data(box,robot_name)
        # self.cluster_points(thresh = 30)
        scaled_polygon_pcl = PointCloud2()
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(robot_boxes.header, self.points)
        #print(scaled_polygon_pcl)
        open3d_cloud = o3d.geometry.PointCloud()

        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(self.points))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(self.color)/255.0)

        rospack = rospkg.RosPack()
        o3d.visualization.draw_geometries([open3d_cloud])
        o3d.io.write_point_cloud(rospack.get_path('src2_object_detection')+"/pcl/perpective_01.pcd", open3d_cloud)
#        target = o3d.io.read_point_cloud(rospack.get_path('src2_object_detection')+"/pcl/repair_station.pcd")


            #voxel_down_pcd = open3d_cloud.voxel_down_sample(voxel_size=0.02)
            #print("Statistical oulier removal")
        #cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,
        #                                                std_ratio=2.0)
        #display_inlier_outlier(voxel_down_pcd, ind)
        self.point_cloud_publishers[robot_name].publish(scaled_polygon_pcl)

    def process_data(self, bounding_box,robot_name):
        """
        Convert image to numpy array using opencv bridge and
        get camera parameters. Loop through the bounding box
        coordinates to calculate the 3D points
        """
        self.bounding_box = bounding_box
        self.disparity_image = self.disparity_images[robot_name]
        self.rgb = self.rgb_images[robot_name]

        self.bridge = CvBridge()
        self.image = self.bridge.imgmsg_to_cv2(self.disparity_image.image, desired_encoding='passthrough')
        self.image_rgb = self.bridge.imgmsg_to_cv2(self.rgb, "rgb8")

        self.cx = float(self.disparity_image.valid_window.width+1)/2
        self.cy = float(self.disparity_image.valid_window.height+1)/2
        self.sx = self.disparity_image.f #Focal length
        self.sy = self.disparity_image.f
        self.bl = self.disparity_image.T # baseline
        for x in range(self.bounding_box.xmin, self.bounding_box.xmax):
            for y in range(self.bounding_box.ymin,self.bounding_box.ymax):
                disparity = self.image.transpose()[x,y] #get x and y on the correct indices
                rgb = self.image_rgb.transpose()[:,x,y]
                self.ipt_to_opt(disparity,rgb, x,y)

    def ipt_to_opt(self, disparity,rgb, x,y):
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
        self.color.append(rgb)

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

    def shutdown(self):
        """
        Shutdown Node
        """
        rospy.loginfo("Obstacle Detection Node is shutdown")
        rospy.sleep(3)

def main():
    try:
        rospy.init_node('obstacles_to_point_cloud',anonymous=True)
        obstacles_to_point_cloud = ObstaclesToPointCloudMultipleRovers()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
