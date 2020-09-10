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
from driving_tools.srv import CirculateBaseStation
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
        rospy.loginfo("Align Base Station service node is running")
        s = rospy.Service('align_base_station', AlignBaseStation, self.align_base_station_handle)
        self.marker = []
        rospy.spin()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera and classfier
        """
        left_img_sub = message_filters.Subscriber("camera/left/image_raw", Image)
        right_img_sub = message_filters.Subscriber("camera/right/image_raw", Image)
        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        ts = message_filters.ApproximateTimeSynchronizer([left_img_sub,right_img_sub,boxes_sub], 10, 0.1, allow_headerless=True)
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
        #self.range = req.range.data
        #self.true_range = self.laser_mean()
        self.true_range = req.range.data
        self.range = self.true_range+1.9
        print("True Range" +str(self.true_range))
        response_ = self.align()
        response = AlignBaseStationResponse()
        resp_ = Bool(response_)
        response.success = resp_
        return response

    def align(self):
        """
        Turn around base station maintaining true range using direct laser data
        until marker is in the center of the image
        """
        counter = 0
        while not self.marker_centered():
            curr_dist_ = self.laser_mean()
            if curr_dist_:
                range_ = curr_dist_
            print(range_)
            print(self.range)
            print(self.true_range)
            range_ = self.range + self.true_range - range_
            print(range_)
            self.circulate_base_station_service(0.1, range_)
            rospy.sleep(0.5)
            counter +=1
            if counter >15:
                counter = 0
                rospy.sleep(0.2)
                self.face_base()
                self.stop()
        self.stop()
        self.face_marker() # center the marker a last time
        self.stop()
        return True

    def marker_centered(self):
        """

        """
        self.check_for_base_station_marker(self.boxes.boxes)
        if self.marker:
            print( np.abs((self.marker.xmax+self.marker.xmin)/2.0-320)<5.0)
            if np.abs((self.marker.xmax+self.marker.xmin)/2.0-320)<5.0:
                return True
        return False

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

    def face_base(self):
        while True:
            self.check_for_base_station(self.boxes.boxes)
            x_mean = float(self.base.xmin+self.base.xmax)/2.0-320
            print(-x_mean)
            self.drive(0.0, -x_mean/640)
            if np.abs(x_mean)<5:
                break

    def face_marker(self):
        while True:
            self.check_for_base_station_marker(self.boxes.boxes)
            x_mean = float(self.marker.xmin+self.marker.xmax)/2.0-320
            print(-x_mean)
            self.drive(0.0, -x_mean/1000)
            print(self.marker.xmax-self.marker.xmin)
            if np.abs(x_mean)<5:
                break



    def circulate_base_station_service(self, throttle, radius):
        """
        Service
        """
        rospy.loginfo("Call ObjectEstimation Service")
        rospy.wait_for_service('driving/circ_base_station')
        circulate_base_station_service_call = rospy.ServiceProxy('driving/circ_base_station', CirculateBaseStation) # Change the service name when inside launch file
        try:
            circ_base_ = circulate_base_station_service_call(throttle, radius)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Add some exception
        print(circ_base_)
        return circ_base_

    def drive(self, linear_speed, heading, y = 0.0):
        _cmd_publisher = rospy.Publisher(robot_name+"/driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = linear_speed
        _cmd_message.linear.y = y
        _cmd_message.angular.z = heading
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

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

    def check_for_base_station_marker(self,boxes):
        for box in boxes:
            if box.id == 3:
                self.marker = box

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
        rospy.loginfo("Align Base Station service node is shutdown")
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('align_base_station_service',anonymous=True)
        align_base = AlignBaseStationService()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
