#!/usr/bin/env python3

## DEPRECATED ##

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
from src2_object_detection.srv import WhereToParkHauler, WhereToParkHaulerResponse, WhereToParkHaulerRequest
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import std_msgs.msg

import numpy as np
import tf.transformations as t_

DISTANCE = 10 # constant in meters
robot_name = rospy.get_param('robot_name',"small_excavator_1")


class HaulerParkingPosition:
    """
    Find best parking location for Hauler based on excavator camera view.
    """
    def __init__(self):
        rospy.loginfo("[{}] Node for converting obstacles to point cloud using disparity image is on".format(robot_name))
        self.mast_camera_publisher = rospy.Publisher("sensor/yaw/command/position", Float64, queue_size = 10 )
        rospy.on_shutdown(self.shutdown)
        self.start()

    def start(self):
        """
        Method to start the where_to_park_hauler service server
        """
        s = rospy.Service('where_to_park_hauler', WhereToParkHauler, self.hauler_parking_location_service_handler)
        rospy.spin()

    def hauler_parking_location_service_handler(self, request):
        """
        Service Handler to check for the best parking location for the hauler based
        on excavator camera images from left and right
        """
        #Intialize and clear boxes
        self.left_boxes = DetectedBoxes()
        self.right_boxes = DetectedBoxes()

        #Turn left:
        print("Turning Left (10seconds)")
        self.mast_camera_publisher.publish(np.pi/2.0)
        rospy.sleep(12)
        rospy.wait_for_service('/find_object')
        _find_object =rospy.ServiceProxy('/find_object', FindObject)
        try:
            _find_object = _find_object(robot_name = robot_name)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print(_find_object)
        self.left_boxes = _find_object.boxes

        #Turn Right
        print("Turning Right (20seconds)")
        self.mast_camera_publisher.publish(-np.pi/2.0)
        rospy.sleep(24)
        rospy.wait_for_service('/find_object')
        _find_object =rospy.ServiceProxy('/find_object', FindObject)
        try:
            _find_object = _find_object(robot_name = robot_name)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        self.right_boxes = _find_object.boxes
        # Center camera again
        self.mast_camera_publisher.publish(0.0)
        #Get odom topic and estimate excavator heading
        excavator_odom = rospy.wait_for_message("localization/odometry/sensor_fusion", Odometry)
        self.excavator_pose = excavator_odom.pose.pose
        print(self.excavator_pose)
        excavator_orientation_euler = t_.euler_from_quaternion([self.excavator_pose.orientation.x,
                                                        self.excavator_pose.orientation.y,
                                                        self.excavator_pose.orientation.z,
                                                        self.excavator_pose.orientation.w])
        #Compare sides:
        side = self.compare_sides()
        self.get_left_pose(excavator_orientation_euler[2])
        self.get_right_pose(excavator_orientation_euler[2])
        rospy.loginfo("[{}] Side: {}".format(robot_name, side))
        if side == "left":
            x,y = self.get_left_pose(excavator_orientation_euler[2])
            new_orientation = list(excavator_orientation_euler)
            new_orientation[2] =  new_orientation[2] - np.pi/2.0
            new_orientation_quat = t_.quaternion_from_euler(new_orientation[0],
                                                            new_orientation[1],
                                                            new_orientation[2])
        if side == "right":
            x,y = self.get_right_pose(excavator_orientation_euler[2])
            new_orientation = list(excavator_orientation_euler)
            new_orientation[2] =  new_orientation[2] + np.pi/2.0
            new_orientation_quat = t_.quaternion_from_euler(new_orientation[0],
                                                            new_orientation[1],
                                                            new_orientation[2])
        rospy.loginfo("[{}] Parking Position - x,y: {},{} - Perpendicular Orientation: {}".format(robot_name, x, y, new_orientation[2]))

        best_position = WhereToParkHaulerResponse()
        best_position.pose.position.x = x
        best_position.pose.position.y = y
        best_position.pose.orientation.x = new_orientation_quat[0]
        best_position.pose.orientation.y = new_orientation_quat[1]
        best_position.pose.orientation.z = new_orientation_quat[2]
        best_position.pose.orientation.w = new_orientation_quat[3]
        if side == "left":
            best_position.side = 1
        if side == "right":
            best_position.side = -1
        #best_position.pose.orientation = new_orientation_quat
        return(best_position)

    def compare_sides(self):
        """
        Simple comparison where the side with least number of obstacle is chosen
        """
        left_boxes = []
        right_boxes = []
        for box in self.right_boxes.boxes:
            if box.id == 5:
                right_boxes.append(box)
        for box in self.left_boxes.boxes:
            if box.id == 5:
                left_boxes.append(box)
        if len(left_boxes) > len(right_boxes):
            return "right"
        else:
            return "left"

    def get_right_pose(self, heading):
        """
        Equations to estimate the x,y position of the hauler in the right side given
        the current position of the excavator
        """
        x = self.excavator_pose.position.x + DISTANCE*np.sin(heading)
        y = self.excavator_pose.position.y + DISTANCE*np.cos(heading+np.pi)
        print(x)
        print(y)
        return x,y

    def get_left_pose(self, heading):
        """
        Equations to estimate the x,y position of the hauler in the left side given
        the current position of the excavator
        """
        x = self.excavator_pose.position.x - DISTANCE*np.sin(heading)
        y = self.excavator_pose.position.y + DISTANCE*np.cos(heading)
        return x,y

    def shutdown(self):
        """
        Shutdown Node
        """
        rospy.loginfo("[{}] Hauler Parking Position Node is shutdown".format(robot_name))
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('hauler_parking_position',anonymous=True)
        obstacles_to_point_cloud = HaulerParkingPosition()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
