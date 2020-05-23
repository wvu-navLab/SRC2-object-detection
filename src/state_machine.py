#!/usr/bin/env python
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import object_estimation, object_estimationResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from srcp2_msgs.srv import LocalizationSrv, AprioriLocationSrv
import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf import TransformListener, TransformBroadcaster
import tf.transformations as t_
import numpy as np

robot_name = "/scout_1"
robot_base_frame = "scout_1_tf/base_footprint"
robot_disparity_image_camera_frame = "scout_1_tf/left_camera_optical"

class StateMachineRound3:
    """

    """
    def __init__(self):
        """
        """
        rospy.loginfo("Object 3D point estimation node is running")
        self.cube_sat = False #variable used later for checking if cubesat was found
        self.tf = TransformListener()
        self.br = TransformBroadcaster()

        self.stereo_subscriber()
        rospy.sleep(5)
        self.wait_for_cubesat()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        left_img_sub = message_filters.Subscriber(robot_name+"/camera/left/image_raw", Image)
        right_img_sub = message_filters.Subscriber(robot_name+"/camera/right/image_raw", Image)
        disparity_sub = message_filters.Subscriber("/disparity", DisparityImage)
        boxes_sub = message_filters.Subscriber("/DetectedBoxes", DetectedBoxes)
        ts = message_filters.TimeSynchronizer([left_img_sub,right_img_sub,disparity_sub,boxes_sub],5)
        ts.registerCallback(self.image_callback)

    def image_callback(self,left_img, right_img, disparity,boxes):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.left_img = left_img
        self.right_img = right_img
        self.disparity = disparity
        self.boxes = boxes


    def wait_for_cubesat(self):
        """
        Method that keeps checking to see if the cubesat was found on the images
        """
        while True:
            self.check_for_cube_sat(self.boxes.boxes)
            if self.cube_sat:
                self.object_pose_estimation_service_call()
                break
        self.get_true_pose()
        p,q = self.get_transform()
        self.report_cubesat(p)

    def get_transform(self):
        p,q = self.get_transform_frame(robot_disparity_image_camera_frame, robot_base_frame)
        cameraHcube_ = np.eye(4)
        cameraHcube_[0:3,3] = [self.cube_3d_point.object_position.point.x,
                            self.cube_3d_point.object_position.point.y,
                                self.cube_3d_point.object_position.point.z]
#        print(cameraHcube_)
        baseHcamera_ = t_.quaternion_matrix(q)
        baseHcamera_[0:3,3] = p
#        print(baseHcamera_)
        globalHbase_ = t_.quaternion_matrix([self.global_pose_rover_base_frame.pose.orientation.x,
                                            self.global_pose_rover_base_frame.pose.orientation.y,
                                            self.global_pose_rover_base_frame.pose.orientation.z,
                                            self.global_pose_rover_base_frame.pose.orientation.w])

        globalHbase_[0:3,3] = [self.global_pose_rover_base_frame.pose.position.x,
                              self.global_pose_rover_base_frame.pose.position.y,
                              self.global_pose_rover_base_frame.pose.position.z]
#        print(globalHbase_)
        pose_global_ = np.matmul(globalHbase_,np.matmul(baseHcamera_,cameraHcube_))
#        print(pose_global_)
        time_ = rospy.Time.now()
        self.br.sendTransform((self.cube_3d_point.object_position.point.x,
                            self.cube_3d_point.object_position.point.y,
                                self.cube_3d_point.object_position.point.z),
                    (0.0,0.0,0.0,1.0),
                     time_,
                     "cube_sat",
                     "scout_1_tf/left_camera_optical"
                     )

        self.br.sendTransform((self.global_pose_rover_base_frame.pose.position.x,
                              self.global_pose_rover_base_frame.pose.position.y,
                              self.global_pose_rover_base_frame.pose.position.z),
                            (self.global_pose_rover_base_frame.pose.orientation.x,
                             self.global_pose_rover_base_frame.pose.orientation.y,
                             self.global_pose_rover_base_frame.pose.orientation.z,
                             self.global_pose_rover_base_frame.pose.orientation.w),
                             time_,
                             "scout_1_tf/base_footprint",
                             "world"
                             )
        p,q = self.get_transform_frame("cube_sat", "world")
        rospy.loginfo("Cubesat Global Pose")
        print(p,q)
        return (p,q)


    def check_for_cube_sat(self,boxes):
        for box in boxes:
            if box.id == 1:
                self.cube_sat = box

    def object_pose_estimation_service_call(self):
        """
        Service that returns the 3D point from the cubesat bounding box
        disparity and stereo image
        """
        rospy.loginfo("Call ObjectEstimation Service")
        rospy.wait_for_service('Cubesat_point_estimation')
        object_estimation_call = rospy.ServiceProxy('Cubesat_point_estimation', object_estimation)
        try:
            self.cube_3d_point = object_estimation_call(self.cube_sat, self.disparity, self.left_img, self.right_img)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        rospy.loginfo("Cubesat point with respect to camera frame")
        print(self.cube_3d_point)

    def get_true_pose(self):
        """
        """
        get_true_pose_call = rospy.ServiceProxy('/scout_1/get_true_pose', LocalizationSrv)
        try:
            self.global_pose_rover_base_frame = get_true_pose_call(True)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

#        pose = Pose()
#        pose.position.x = -23.4600716594
#        pose.position.y = 8.51154641996
#        pose.position.z = 0.676950941171
#        pose.orientation.x = -0.00663293319829
#        pose.orientation.y = -0.00826738406403
#        pose.orientation.z = -0.0756061400248
#        pose.orientation.w = 0.997081424031
#        self.global_pose_rover_base_frame = pose
        rospy.loginfo("Rover Global Pose")
        print(self.global_pose_rover_base_frame)

    def get_transform_frame(self, source, dest):
        self.tf.waitForTransform(source, dest, rospy.Time(), rospy.Duration(4.0))
        t = self.tf.getLatestCommonTime(source, dest)
        position, quaternion = self.tf.lookupTransform(dest, source, t)
        return position, quaternion

    def report_cubesat(self, point):
        p_ = Point()
        p_.x, p_.y, p_.z = point
        report_service_call = rospy.ServiceProxy('apriori_location_service', AprioriLocationSrv)
        try:
            report_service_call = report_service_call(p_)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print(report_service_call)

def main():
    try:
        rospy.init_node('state_machine_test_round3',anonymous=True)
        object_estimation_service_call = StateMachineRound3()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
