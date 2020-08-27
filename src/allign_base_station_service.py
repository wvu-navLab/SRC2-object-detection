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
from src2_object_detection.srv import AlignBaseStation, AlignBaseStationResponse
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

class AlignBaseStationService:
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
        s = rospy.Service('align_base_station', AlignBaseStation, self.align_base_station_handle)
        rospy.spin()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera and classfier
        """
        left_img_sub = message_filters.Subscriber(robot_name+"/camera/left/image_raw", Image)
        right_img_sub = message_filters.Subscriber(robot_name+"/camera/right/image_raw", Image)
        boxes_sub = message_filters.Subscriber("/DetectedBoxes", DetectedBoxes)
        ts = message_filters.TimeSynchronizer([left_img_sub,right_img_sub,boxes_sub],5)
        ts.registerCallback(self.image_callback)

    def image_callback(self,left_img, right_img, boxes):
        """
        Subscriber callback for the stereo camera, with synchronized images and
        classifier ounding boxes
        """
        self.left_img = left_img
        self.right_img = right_img
        self.boxes = boxes

    def align_base_station_handle(self, req):
        """
        Service for aliging the rover with the base station marker
        (without calling the service to score points in qualification round 3)
        """
        rospy.loginfo("Align Base Station Service Started")
        self.range = req.range`
        self.true_range = self.laser_mean()
        print("True Range" +str(self.true_range))
        response = self.align()
        return response

    def align(self):
        """
        Turn around base station maintaining true range using direct laser data
        until marker is in the center of the image
        """
        while not self.marker_centered():
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

    def marker_centered(self):
        """

        """
        return False

    def turn_in_place(self, direction):
        """
        Turn in place clockwise or counter clockwise
        """
        _cmd_publisher = rospy.Publisher(robot_name+"/driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.angular.z = 0.5*direction
        for i in range(2):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def circulate_base_station_service_call(self, throttle, radius):
        """
        Service
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

rosservice call /scout_1/driving/circ_base_station "throttle: 0.0
radius: 0.0"

    def stop(self):
        _cmd_publisher = rospy.Publisher(robot_name+"/driving/cmd_vel", Twist, queue_size = 10 )
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
        laser = rospy.wait_for_message(robot_name+"/laser/scan", LaserScan)
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
