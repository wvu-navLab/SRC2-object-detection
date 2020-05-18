#!/usr/bin/env python

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

from keras import backend as K
from keras.models import load_model
from keras.preprocessing import image
from keras.optimizers import Adam

from models.keras_ssd300 import ssd_300
from keras_loss_function.keras_ssd_loss import SSDLoss
from keras_layers.keras_layer_AnchorBoxes import AnchorBoxes
from keras_layers.keras_layer_DecodeDetections import DecodeDetections
from keras_layers.keras_layer_DecodeDetectionsFast import DecodeDetectionsFast
from keras_layers.keras_layer_L2Normalization import L2Normalization

from ssd_encoder_decoder.ssd_output_decoder import decode_detections, decode_detections_fast

from data_generator.object_detection_2d_data_generator import DataGenerator
from data_generator.object_detection_2d_photometric_ops import ConvertTo3Channels
from data_generator.object_detection_2d_geometric_ops import Resize
from data_generator.object_detection_2d_misc_utils import apply_inverse_transforms
import tensorflow as tf

# System Libraries
import glob
import os

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


class Object_Detection_Test:
    def __init__(self):
        """
        Initialize the box and detected boxes publisher, and define the Subscriber
        for the
        """
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Object Detection Test Started")
        self.stereo_subscriber()
        self.box_pub = rospy.Publisher("Box", Box,queue_size=1)
        self.boxes_pub = rospy.Publisher("DetectedBoxes", DetectedBoxes, queue_size = 1)
        K.clear_session() # Clear previous models from memory.
        self.model = ssd_300(image_size=(img_height, img_width, 3),
                n_classes=2,
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
        weights_path = "src2_test01.h5"
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
        left_img_sub = message_filters.Subscriber("/scout_1/camera/left/image_raw", Image)
        left_cam_info_sub = message_filters.Subscriber("/scout_1/camera/left/camera_info", CameraInfo)
        right_img_sub = message_filters.Subscriber("/scout_1/camera/right/image_raw", Image)
        right_cam_info_sub = message_filters.Subscriber("/scout_1/camera/right/camera_info", CameraInfo)
        ts = message_filters.TimeSynchronizer([left_img_sub,left_cam_info_sub,right_img_sub,right_cam_info_sub],5)
        ts.registerCallback(self.image_callback)

    def image_callback(self,left_img,left_cam_info, right_img, right_cam_info):
        """
        Subscriber callback for the stereo camera
        """
        self.left_img = left_img
        self.left_cam_info = left_cam_info
        self.right_img = right_img
        self.right_cam_info = right_cam_info


    def start(self):
        plt.ion()
        while not rospy.is_shutdown():
            #box = Box()
            #box.id = 1
            #box.xmin = np.random.randint(0,500)
            #box.ymin = np.random.randint(0,500)
            #box.xmax = np.random.randint(0,500)
            #box.ymax = np.random.randint(0,500)
            #self.box_pub.publish(box)
            ##print(self.left_img.header)
            #boxes = DetectedBoxes()
            ##boxes.header = self.left_img.header
            #boxes.boxes = [box,box,box,box]
            #self.boxes_pub.publish(boxes)

            self.bridge = CvBridge()
            original_image = self.bridge.imgmsg_to_cv2(self.left_img, "rgb8")
            resized_image = cv2.resize(original_image, dsize=(300, 300), interpolation=cv2.INTER_CUBIC)
            print(resized_image.shape)

            y_pred = self.model.predict(resized_image.reshape(1,300,300,3))
            y_pred_thresh = [y_pred[k][y_pred[k,:,1] > confidence_threshold] for k in range(y_pred.shape[0])]

            np.set_printoptions(precision=2, suppress=True, linewidth=90)
            print("Predicted boxes:\n")
            print('   class   conf xmin   ymin   xmax   ymax')
            print(y_pred_thresh[0])
            colors = plt.cm.hsv(np.linspace(0, 1, 21)).tolist()
            classes = ['background',
                'cubesat', 'base station']

            plt.ion()
            plt.figure(figsize=(20,12))
            plt.imshow(original_image)

            current_axis = plt.gca()
            for box in y_pred_thresh[0]:
                    # Transform the predicted bounding boxes for the 300x300 image to the original image dimensions.
                xmin = box[2] * original_image.shape[1] / img_width
                ymin = box[3] * original_image.shape[0] / img_height
                xmax = box[4] * original_image.shape[1] / img_width
                ymax = box[5] * original_image.shape[0] / img_height
                color = colors[int(box[0])]
                label = '{}: {:.2f}'.format(classes[int(box[0])], box[1])
                current_axis.add_patch(plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, color=color, fill=False, linewidth=2))
                current_axis.text(xmin, ymin, label, size='x-large', color='white', bbox={'facecolor':color, 'alpha':1.0})
            plt.show()

            plt.close()


    def shutdown(self):
        rospy.loginfo("Object Detection Test is shutdown")
        rospy.sleep(1)

def main():
    try:
    	rospy.init_node('object_detection_test', anonymous=True)
        object_detection_test = Object_Detection_Test()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
