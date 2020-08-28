#!/usr/bin/env python
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import ApproachBaseStation, ApproachBaseStationResponse
from range_to_base.srv import RangeToBase, RangeToBaseResponse

from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
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

class ApproachBaseStationService:
    """

    """
    def __init__(self):
        """
        """
        rospy.on_shutdown(self.shutdown)
        self.base = False
        self.stereo_subscriber()
        rospy.sleep(2)
        rospy.loginfo("Approach Base Station service node is running")
        s = rospy.Service('approach_base_station', ApproachBaseStation, self.approach_base_station_handle)
        rospy.spin()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        left_img_sub = message_filters.Subscriber("camera/left/image_raw", Image)
        right_img_sub = message_filters.Subscriber("camera/right/image_raw", Image)
        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        ts = message_filters.TimeSynchronizer([left_img_sub,right_img_sub,boxes_sub],5)
        ts.registerCallback(self.image_callback)

    def image_callback(self,left_img, right_img, boxes):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.left_img = left_img
        self.right_img = right_img
        self.boxes = boxes

    def approach_base_station_handle(self, req):
        """
        Service for approaching the base station in the SRC qualification
        (without calling the service to score points in qualification round 3)
        """
        response = self.search_for_base_station()
        rospy.loginfo("Aprroach Base Station Service Started")
        return response

    def search_for_base_station(self):
        """
        Turn in place to check for base station
        """
        search_ = False
        for i in range(150):
            self.turn_in_place(-1)
            self.check_for_base_station(self.boxes.boxes)
            if self.base:
                print("Base Station found")
                print(self.base)
                self.stop()
                range_ = self.approach_base_station()
                search_ = True
                break
        self.stop()
        response = approach_base_stationResponse()
        resp_ = Bool()
        resp_.data = search_
        response.success = resp_
        if search_ == True:
            rospy.loginfo("Rover approached base station")
            base_station_range = Float64()
            base_station_range.data =range_.range.range
            response.range = base_station_range
        else:
            rospy.logerr("Base Station was not found when running turn in place maneuver")
        self.base = False # reset flag variable
        return response

    def approach_base_station(self):
        """
        Visual approach the base station,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        while True:
            self.check_for_base_station(self.boxes.boxes)
            x_mean = float(self.base.xmin+self.base.xmax)/2.0-320
            print(-x_mean/640)
            self.drive(0/05, -x_mean/640)
            print(self.base.xmax-self.base.xmin)
            print(self.laser_mean())
            if (self.base.xmax-self.base.xmin) > 350 and self.laser_mean() < 6:
                break
        print("Close to base station")
        self.stop()
        range_ = self.range_base_service_call()
        return range_

    def turn_in_place(self, direction):
        """
        Turn in place clockwise or counter clockwise
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.angular.z = 0.5*direction
        for i in range(2):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def drive_straight(self, time):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = 1
        rospy.loginfo("test")
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)
        rospy.sleep(time)

    def drive(self,time, heading):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = 1
        _cmd_message.angular.z = heading
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)
        rospy.sleep(time)


    def stop(self):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_publisher.publish(_cmd_message)

    def range_base_service_call(self):
        """
        Service that returns the 3D point from the cubesat bounding box
        disparity and stereo image
        """
        rospy.loginfo("Call ObjectEstimation Service")
        rospy.wait_for_service('range_to_base_service')
        range_to_base_call = rospy.ServiceProxy('range_to_base_service', RangeToBase) # Change the service name when inside launch file
        try:
            range_to_base_call = range_to_base_call(0.4)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Add some exception
        print(range_to_base_call)
        return range_to_base_call

    def check_for_base_station(self,boxes):
        for box in boxes:
            if box.id == 2:
                self.base = box

    def laser_mean(self):
        laser = rospy.wait_for_message("laser/scan", LaserScan)
        #print(laser)
        #print("average:")
        #print(np.mean(laser.ranges))
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
        rospy.loginfo("Approach Base Station service node is shutdown")
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('approach_base_station_service',anonymous=True)
        object_estimation_service_call = ApproachBaseStationService()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
