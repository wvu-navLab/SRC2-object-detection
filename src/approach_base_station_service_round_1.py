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
from src2_object_detection.srv import ObjectEstimation, ObjectEstimationResponse
from src2_object_detection.srv import ApproachBaseStation, ApproachBaseStationResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, AprioriLocationSrv, ToggleLightSrv
import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf import TransformListener, TransformBroadcaster
import tf.transformations as t_
import numpy as np


class Obstacle:
    """
    Obstacle Definition
    """
    def __init__(self,obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class ApproachBaseStationService:
    """

    """
    def __init__(self):
        """
        """
        rospy.on_shutdown(self.shutdown)
        self.base = False
        self.obstacles = []
        self.timeout = 60
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
        disparity_sub = message_filters.Subscriber("disparity", DisparityImage)
        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        ts = message_filters.ApproximateTimeSynchronizer([disparity_sub, boxes_sub],10, 0.1, allow_headerless=True)

        ts.registerCallback(self.image_callback)

    def image_callback(self, disparity, boxes):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.obstacles = []
        self.disparity = disparity
        self.boxes = boxes
        self.check_for_obstacles(self.boxes.boxes)

        for obstacle_ in self.obstacle_boxes:
            dist_ = self.object_distance_estimation(obstacle_)
            self.obstacles.append(Obstacle(obstacle_,dist_.object_position.point.z))

    def approach_base_station_handle(self, req):
        """
        Service for approaching the base station in the SRC qualification
        (without calling the service to score points in qualification round 1)
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
                range_, search_ = self.approach_base_station()
                print("SEARCH")
                print(search_)
                if search_ == True:
                    self.face_base()
                self.stop()
                break

        response = ApproachBaseStationResponse()
        resp_ = Bool()
        resp_.data = search_
        response.success = resp_
        base_station_range = Float64()
        base_station_range.data = range_
        response.range = base_station_range

        if search_ == True:
            rospy.loginfo("Rover approached base station")
        else:
            rospy.logerr("Base Station was not found when running turn in place maneuver or timeout")
        self.base = False # reset flag variable
        return response

    def approach_base_station(self):
        """
        Visual approach the base station,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        turning_offset_i_ = 0.0
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time_ = rospy.get_time()

        toggle_light_ = 1

        while True:
            self.check_for_base_station(self.boxes.boxes)
            x_mean_base = float(self.base.xmin+self.base.xmax)/2.0-320
            minimum_dist_ = 10.0
            turning_offset = 0.0
            laser_ = self.laser_mean()

            if laser_ <10.0:
                rospy.logerr("MIN DIST SET TO 6")
                minimum_dist_ = 6.0

            if toggle_light_ == 1:
                self.toggle_light(0.6)
                toggle_light_ = 0
            else:
                self.toggle_light(0.0)
                toggle_light_ = 1

            curr_time_ = rospy.get_time()
            print("TIME")
            print(curr_time_ - init_time_)
            if curr_time_ - init_time_ >50:
                rospy.logerr("Time Out in Approach Base Station")
                return 0.0, False
            print("test")

            for obstacle_ in self.obstacles:
                obstacle_mean_ = float(obstacle_.obstacle.xmin+obstacle_.obstacle.xmax)/2.0-320
                turning_offset_i_ = turning_offset
                if obstacle_.distance > 0.1:
                    if obstacle_.distance < minimum_dist_:
                        minimum_dist_ = obstacle_.distance
                    if obstacle_.distance < 8:
                        turning_offset += np.sign(obstacle_mean_)*0.3*(1-np.abs(obstacle_mean_)/320.0)
#                print(obstacle_.distance)
            speed = minimum_dist_/10.0
            rotation_speed = -x_mean_base/840+turning_offset+0.5*turning_offset_i_

            # if turning_offset != 0.0:
            #  r = rospy.Rate(10)
            #  print("AVOIDING!!!!!")
            #  for i in range(20):
            #      self.drive_crab_motion(speed*0.1, rotation_speed*5)
            #      r.sleep()
            self.drive(speed, rotation_speed)
            if (self.base.xmax-self.base.xmin) > 340 and laser_ < 5.6:
                break
        print("Close to base station")
        self.stop()
        return self.laser_mean(), True
        #range_ = self.range_base_service_call()

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

    def object_distance_estimation(self, object):

        rospy.wait_for_service('cubesat_point_estimation') #Change the name of the service
        object_estimation_call = rospy.ServiceProxy('cubesat_point_estimation', ObjectEstimation)
        try:
            object_distance = object_estimation_call(object, self.disparity)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return(object_distance)

    def drive_straight(self, time):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = 1
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)
        rospy.sleep(time)

    def drive(self,speed, heading):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = speed
        _cmd_message.angular.z = heading
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def drive_crab_motion(self,x_speed, y_speed):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = x_speed
        _cmd_message.linear.y = y_speed
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)


    def stop(self):
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 1 )
        _cmd_message = Twist()
        _cmd_publisher.publish(_cmd_message)
        _cmd_publisher.publish(_cmd_message)

    def check_for_base_station(self,boxes):
        for box in boxes:
            if box.id == 2:
                self.base = box

    def check_for_obstacles(self,boxes):
        self.obstacle_boxes = []
        for box in boxes:
            if box.id == 4:
                self.obstacle_boxes.append(box)

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

    def face_base(self):
        while True:
            self.check_for_base_station(self.boxes.boxes)
            x_mean = float(self.base.xmin+self.base.xmax)/2.0-320
            print(-x_mean)
            self.drive(0.0, -x_mean/640)
            print(self.base.xmax-self.base.xmin)
            if np.abs(x_mean)<5:
                break

    def toggle_light(self, value):
        toggle_light_call = rospy.ServiceProxy('toggle_light', ToggleLightSrv)
        try:
            toggle_light_call = toggle_light_call(str(value))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


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
