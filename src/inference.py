#!/usr/bin/env python

"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
# ROS Libraries
import rospy
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
import numpy as np
import rospkg

from keras import backend as K
from keras.models import load_model
from keras.preprocessing import image
from keras.optimizers import Adam
from models.keras_ssd300 import ssd_300
from keras_loss_function.keras_ssd_loss import SSDLoss

import tensorflow as tf

# System Libraries
import glob
import os

#robot name (make it as a ros param later) ****************************************
robot_name = "/scout_1"

#
# Set the image size.
img_height = 300
img_width = 300
scales = [0.07, 0.15, 0.33, 0.51, 0.69, 0.87, 1.05] # The anchor box scaling factors used in the original SSD300 for the MS COCO datasets.
# scales = [0.1, 0.2, 0.37, 0.54, 0.71, 0.88, 1.05] # The anchor box scaling factors used in the original SSD300 for the Pascal VOC datasets.
aspect_ratios = [[1.0, 2.0, 0.5],
                 [1.0, 2.0, 0.5, 3.0, 1.0/3.0],
                 [1.0, 2.0, 0.5, 3.0, 1.0/3.0],
                 [1.0, 2.0, 0.5, 3.0, 1.0/3.0],
                 [1.0, 2.0, 0.5],
                 [1.0, 2.0, 0.5]] # The anchor box aspect ratios used in the original SSD300; the order matters
two_boxes_for_ar1 = True
steps = [8, 16, 32, 64, 100, 300] # The space between two adjacent anchor box center points for each predictor layer.
offsets = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5] # The offsets of the first anchor box center points from the top and left borders of the image as a fraction of the step size for each predictor layer.
clip_boxes = False # Whether or not you want to limit the anchor boxes to lie entirely within the image boundaries
variances = [0.1, 0.1, 0.2, 0.2] # The variances by which the encoded target coordinates are scaled as in the original implementation
normalize_coords = True
swap_channels = [2, 1, 0]
subtract_mean = [123, 117, 104] # get the mean for this dataset

confidence_threshold = 0.5 # Thershold for the box estimation


class Object_Detection_Inference:
    def __init__(self):
        """
        Initialize the bounding boxes publisher, define subscriber for the image
        topics and initialize the SSD300 inference model with the parameters defined
        above
        """
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Object Detection Inference Started")
        self.stereo_subscriber()
        rospack = rospkg.RosPack()
        self.box_pub = rospy.Publisher("Box", Box,queue_size=1)
        self.boxes_pub = rospy.Publisher("DetectedBoxes", DetectedBoxes, queue_size = 1)
        K.clear_session() # Clear previous models from memory.
        self.model = ssd_300(image_size=(img_height, img_width, 3),
                n_classes=7,
                mode='inference',
                l2_regularization=0.0005,
                scales=scales,
                aspect_ratios_per_layer=aspect_ratios,
                two_boxes_for_ar1=two_boxes_for_ar1,
                steps=steps,
                offsets=offsets,
                clip_boxes=clip_boxes,
                variances=variances,
                normalize_coords=normalize_coords,
                subtract_mean=subtract_mean,
                swap_channels=swap_channels)
        #Load some weights into the model.
        weights_path = rospack.get_path('src2_object_detection')+"/src/src2_hackaton01_weights.h5"
        self.model.load_weights(weights_path, by_name=True)
        # 3: Instantiate an optimizer and the SSD loss function and compile the model.
        #    If you want to follow the original Caffe implementation, use the preset SGD
        #    optimizer, otherwise I'd recommend the commented-out Adam optimizer.
        adam = Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)
        #sgd = SGD(lr=0.001, momentum=0.9, decay=0.0, nesterov=False)
        ssd_loss = SSDLoss(neg_pos_ratio=3, alpha=1.0)
        self.model.compile(optimizer="Adam", loss=ssd_loss.compute_loss)
        self.model._make_predict_function()
        self.model.summary()        #self.start()
        self.start()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        left_img_sub = message_filters.Subscriber(robot_name+"/camera/left/image_raw", Image)
        left_cam_info_sub = message_filters.Subscriber(robot_name+"/camera/left/camera_info", CameraInfo)
        right_img_sub = message_filters.Subscriber(robot_name+"/camera/right/image_raw", Image)
        right_cam_info_sub = message_filters.Subscriber(robot_name+"/camera/right/camera_info", CameraInfo)
        ts = message_filters.TimeSynchronizer([left_img_sub,left_cam_info_sub,right_img_sub,right_cam_info_sub],5)
        ts.registerCallback(self.image_callback)

    def image_callback(self,left_img,left_cam_info, right_img, right_cam_info):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.left_img = left_img
        self.left_cam_info = left_cam_info
        self.right_img = right_img
        self.right_cam_info = right_cam_info


    def start(self):
        """
            Loop through transforming the subscribed img_msg to
            an array, resizing it to the 300x300 network input formatself.
            Then running the inference on the network model and publishing
            the bounding boxes as a custom msg
        """
        while not rospy.is_shutdown():
            self.bridge = CvBridge()
            original_image = self.bridge.imgmsg_to_cv2(self.left_img, "rgb8")
            resized_image = cv2.resize(original_image, dsize=(300, 300), interpolation=cv2.INTER_CUBIC)
            boxes = DetectedBoxes()
            boxes.header = self.left_img.header
            y_pred = self.model.predict(resized_image.reshape(1,300,300,3))
            y_pred_thresh = [y_pred[k][y_pred[k,:,1] > confidence_threshold] for k in range(y_pred.shape[0])]
            np.set_printoptions(precision=2, suppress=True, linewidth=90)
            print("Predicted boxes:\n")
            print('   class   conf xmin   ymin   xmax   ymax')
            print(y_pred_thresh[0])
            classes = ['background', 'cube_sat', 'base_station',
            'base_station_marker','obstacle', 'volatile',
            'crater','rover']
            boxes.boxes = []
            for box in y_pred_thresh[0]:
                box_ = Box()
                box_.id = box[0]
                box_.confidence = box[1]
                # Transform the predicted bounding boxes for the 300x300 image to the original image dimensions.
                box_.xmin = box[2] * original_image.shape[1] / img_width
                box_.ymin = box[3] * original_image.shape[0] / img_height
                box_.xmax = box[4] * original_image.shape[1] / img_width
                box_.ymax = box[5] * original_image.shape[0] / img_height
                boxes.boxes.append(box_)
            self.boxes_pub.publish(boxes)

    def shutdown(self):
        """
        Shutdown Node
        """
        rospy.loginfo("Object Detection Inference is shutdown")
        rospy.sleep(1)

def main():
    try:
    	rospy.init_node('object_detection_inference', anonymous=True)
        object_detection_test = Object_Detection_Inference()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
