#!/usr/bin/env python
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
import rospy
from std_msgs.msg import Bool
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import object_estimation, object_estimationResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2

robot_name = "/scout_1"

class EstimationCall:
    """
    Service that takes disparity image, stereo image pair and bounding box and
    returns the average 3D point of that
    """
    def __init__(self):
        rospy.loginfo("Object 3D point estimation node is running")
        self.cube_sat = False #variable used later for checking if cubesat was found
        self.stereo_subscriber()
        rospy.sleep(10)
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

    def check_for_cube_sat(self,boxes):
        for box in boxes:
            if box.id == 1:
                self.cube_sat = box

    def wait_for_cubesat(self):
        """
        Method that keeps checking to see if the cubesat was found on the images
        """
        while True:
            self.check_for_cube_sat(self.boxes.boxes)
            if self.cube_sat:
                self.rossservice_call()
                break

    def rossservice_call(self):
        """
        Service that returns the 3D point from the cubesat bounding box
        disparity and stereo image
        """
        rospy.loginfo("Call ObjectEstimation Service")
        rospy.wait_for_service('Cubesat_point_estimation')
        object_estimation_call = rospy.ServiceProxy('Cubesat_point_estimation', object_estimation)
        try:
            object_estimation_call = object_estimation_call(self.cube_sat, self.disparity, self.left_img, self.right_img)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Add some exception
        print(object_estimation_call)

def main():
    try:
        rospy.init_node('object_estimation_service_call',anonymous=True)
        object_estimation_service_call = EstimationCall()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
