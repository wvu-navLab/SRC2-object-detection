#!/usr/bin/env python3
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

list_of_robots = rospy.get_param('robots_list', ["small_scout_1", "small_hauler_1", "small_excavator_1"]) #List of robots that are being used

class ObstaclesToPointCloudMultipleRovers:
    """
    Convert obstacles bounding boxes to point cloud
    """
    def __init__(self):
        rospy.loginfo("Node for converting obstacles to point cloud using disparity image is on")
        rospy.on_shutdown(self.shutdown)
        self.rgb_images = {key: Image() for key in list_of_robots}
        self.disparity_images = {key: DisparityImage() for key in list_of_robots}
        self.point_cloud_publishers = {key: rospy.Publisher(key+"/inference/point_cloud",
                                        PointCloud2, queue_size = 1 ) for key in list_of_robots}
        self.images_subscriber()
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
        rate = rospy.Rate(1) # ROS Rate at 1Hz
        watch_dog_timer = 0
        robot_boxes = DetectedBoxes()
        while not rospy.is_shutdown():
            for robot in list_of_robots:
                rospy.wait_for_service('/find_object')
                _find_object =rospy.ServiceProxy('/find_object', FindObject)
                try:
                    _find_object = _find_object(robot_name = robot)
                    robot_boxes = _find_object.boxes
                    self.convert_box_to_point_cloud(robot_boxes,robot)
                except rospy.ServiceException as exc:
                    rospy.logerr("Service did not process request: " + str(exc))
            rate.sleep()

    def convert_box_to_point_cloud(self, robot_boxes, robot_name):
        """
        Convert to point cloud and publish - processing plant, charging station,
        rovers, obstacle and processing plant bin
        """
        self.points = []
        for box in robot_boxes.boxes:
            if (box.id == 0 or box.id == 1 or box.id == 2 or
            box.id == 3 or box.id == 4 or box.id == 5 or box.id == 6):
                self.process_data(box,robot_name)
        scaled_polygon_pcl = PointCloud2()
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(robot_boxes.header, self.points)
        self.point_cloud_publishers[robot_name].publish(scaled_polygon_pcl)

    def process_data(self, bounding_box,robot_name):
        """
        Convert image to numpy array using opencv bridge and
        get camera parameters. Loop through the bounding box
        coordinates to calculate the 3D points
        """
        self.bounding_box = bounding_box
        self.disparity_image = self.disparity_images[robot_name]
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
