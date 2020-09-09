#!/usr/bin/env python
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import ObjectEstimation, ObjectEstimationResponse
from src2_object_detection.srv import ApproachBaseStation, ApproachBaseStationResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, AprioriLocationSrv, ToggleLightSrv, BrakeRoverSrv, HomeLocationSrv


import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf import TransformListener, TransformBroadcaster
import tf.transformations as t_
import numpy as np
from scipy.stats import iqr

robot_name = "/scout_1"
robot_base_frame = "scout_1_tf/base_footprint"
robot_disparity_image_camera_frame = "scout_1_tf/left_camera_optical"

class Round3Part1:
    """

    """
    def __init__(self):
        """
        """
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Search for cubesat node running")
        self.cube_sat = False #variable used later for checking if cubesat was found
        self.base = False
        self.boxes = DetectedBoxes()
        self.tf = TransformListener()
        self.br = TransformBroadcaster()
        self.stereo_subscriber()
        rospy.sleep(3)
        self.part1 = False
        self.part2 = False
        self.part3 = False
        self.search_for_cube_sat()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        disparity_sub = message_filters.Subscriber("/disparity", DisparityImage)
        left_img_sub = message_filters.Subscriber("camera/left/image_raw", Image)
        right_img_sub = message_filters.Subscriber("camera/right/image_raw", Image)
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

    def search_for_cube_sat(self):

        cubesat_report_ = self.turn_in_place_cube_sat_search()
        if cubesat_report_ == True:
            print("Points scored")
            self.part1 = True
        else:
            rospy.logerr("CUBE SAT POSITION WAS INCORRECT")
        self.approach_base_station()
        self.stop()
        rospy.sleep(5)
        if self.part1 == True:
            self.score_arrived_home()
        else:
            self.search_for_cube_sat()
            #cubesat_report_ = self.turn_in_place_cube_sat_search()


    def turn_in_place_cube_sat_search(self):
        """
        Turn in place to check for cubesat
        """
        self.toggle_light(0.9)
        _sensor_angle = rospy.Publisher("sensor_controller/command", Float64, queue_size = 100, latch = True)
        _sensor_angle.publish(Float64(1.0))
        resp_ = False
        for i in range(220):
            self.turn_in_place(-1)
            self.check_for_cube_sat(self.boxes.boxes)
            if self.cube_sat:
                print("Cubesat found")
                print(self.cube_sat)
                self.turn_in_place(-1)
                rospy.sleep(0.2)
                if self.cube_sat.confidence > 0.98:
                    self.stop()
                    self.brake_rover(100)
                    rospy.sleep(2)
                    self.object_pose_estimation_service_call()
                else:
                    self.stop()
                    rospy.sleep(1)
                    self.cube_sat = False
                    self.check_for_cube_sat(self.boxes.boxes) # Check if still visualizing cube_sat
                    if self.cube_sat:
                        self.approach_cube_sat()
                        self.stop()
                        self.brake_rover(100)
                        rospy.sleep(2)
                        self.object_pose_estimation_service_call()
                    else:
                        break
                self.get_true_pose()
                p,q = self.get_transform()
                cubesat_report_ = self.report_cubesat(p)
                resp_ = cubesat_report_.result
                self.brake_rover(0)
                _sensor_angle.publish(Float64(0.0))
                break
        _sensor_angle.publish(Float64(0.0))
        return resp_

    def approach_cube_sat(self):

        for i in range(8):
            self.check_for_cube_sat(self.boxes.boxes)
            x_mean = float(self.cube_sat.xmin+self.cube_sat.xmax)/2.0-320
            y_mean = float(self.cube_sat.ymin+self.cube_sat.ymax)/2.0-240
            self.drive(0.0, -x_mean/320)
            print(self.cube_sat.xmax-self.cube_sat.xmin)
        for i in range(175):
            self.check_for_cube_sat(self.boxes.boxes)
            x_mean = float(self.cube_sat.xmin+self.cube_sat.xmax)/2.0-320
            y_mean = float(self.cube_sat.ymin+self.cube_sat.ymax)/2.0-240
            print(-x_mean/640)
            print(y_mean/480)
            if y_mean<0.0:
                x_mean = -x_mean
            self.drive(y_mean/100, -x_mean/1280)
            print(self.cube_sat.xmax-self.cube_sat.xmin)
        self.stop()

    def approach_base_station(self):
        """
        """
        rospy.wait_for_service('approach_base_station')
        approach_base_station_service_call = rospy.ServiceProxy('approach_base_station', ApproachBaseStation)
        try:
            approach_base_station_ = approach_base_station_service_call(Bool(True))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        rospy.loginfo("Base Station Approach")
        #print(approach_base_station_)

    def turn_in_place(self, direction):
        """
        Turn in place clockwise or counter clockwise
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.angular.z = 0.3*direction
        for i in range(2):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def drive_straight(self, time):
        _cmd_publisher = rospy.Publisher(robot_name+"/driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = 1
        rospy.loginfo("test")
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)
        rospy.sleep(time)

    def drive(self, linear_speed, heading):
        _cmd_publisher = rospy.Publisher(robot_name+"/driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = linear_speed
        _cmd_message.angular.z = heading
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def stop(self):
        _cmd_publisher = rospy.Publisher(robot_name+"/driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_publisher.publish(_cmd_message)

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
        rospy.wait_for_service('cubesat_point_estimation')
        x_ = []
        y_ = []
        z_ = []

        for i in range(20):
            object_estimation_call = rospy.ServiceProxy('cubesat_point_estimation', ObjectEstimation)
            try:
                self.cube_3d_point = object_estimation_call(self.cube_sat, self.disparity, self.left_img, self.right_img)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            rospy.loginfo("Cubesat point with respect to camera frame")
            rospy.sleep(0.8)
            x_.append(self.cube_3d_point.object_position.point.x)
            y_.append(self.cube_3d_point.object_position.point.y)
            z_.append(self.cube_3d_point.object_position.point.z)
        self.cube_3d_point.object_position.point.x = np.mean(np.sort(x_)[1:18])
        self.cube_3d_point.object_position.point.y = np.mean(np.sort(y_)[1:18])
        self.cube_3d_point.object_position.point.z = np.mean(np.sort(z_)[1:18])
        print(self.cube_3d_point)



    def get_true_pose(self):
        """
        """
        get_true_pose_call = rospy.ServiceProxy('get_true_pose', LocalizationSrv)
        try:
            self.global_pose_rover_base_frame = get_true_pose_call(True)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
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
        report_service_call = rospy.ServiceProxy('/apriori_location_service', AprioriLocationSrv)
        try:
            report_service_call = report_service_call(p_)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print(report_service_call)
        return report_service_call

    def score_arrived_home(self):

        report_service_call = rospy.ServiceProxy('/arrived_home_service', HomeLocationSrv)
        try:
            report_service_call = report_service_call(True)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print(report_service_call)
        return report_service_call

    def toggle_light(self, value):
        toggle_light_call = rospy.ServiceProxy('toggle_light', ToggleLightSrv)
        try:
            toggle_light_call = toggle_light_call(str(value))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
#        print(toggle_light_call)

    def brake_rover(self, value):
        rospy.wait_for_service('brake_rover')
        braker_rover_call = rospy.ServiceProxy('brake_rover', BrakeRoverSrv)
        try:
            toggle_light_call = braker_rover_call(float(value))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("Rover brake force applied: " + str(value))

    def laser_mean(self):
        laser = rospy.wait_for_message("laser/scan", LaserScan)
        _val = 0
        _ind = 0
        for i in laser.ranges[20:80]:
            if not np.isinf(i):
                _val+=i
                _ind+=1
        print("Laser Range")
        if _ind != 0:
            return(_val/_ind)

    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.loginfo("Object Detection Inference is shutdown")
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('roun3_part1_node',anonymous=True)
        object_estimation_service_call = Round3Part1()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
