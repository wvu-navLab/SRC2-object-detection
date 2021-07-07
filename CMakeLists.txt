################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(src2_object_detection)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  actionlib_msgs
  genmsg
  actionlib
  tf2
  message_runtime
  message_generation
  message_filters
  cv_bridge
  stereo_msgs
  src2_approach_services
)


################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
	FILES
	Box.msg
	DetectedBoxes.msg
)
add_service_files(
  FILES
  ObjectEstimation.srv
  FindCubesat.srv
  FindObject.srv
  WhereToParkHauler.srv
)
generate_messages(
	DEPENDENCIES
  actionlib_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
  stereo_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf message_generation message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ~/srcp2-competitors/ros_workspace/install/include
)


################################################################################
# Install
################################################################################
install(PROGRAMS src/hauler_parking_position_service.py src/inference_service.py src/inference.py src/object_estimation_service.py
src/object_to_point_cloud.py src/obstacle_detection_multiple_rovers.py src/obstacle_detection_open_cv.py src/plot_boxes_real_time.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/bounding_box_utils/bounding_box_utils.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/bounding_box_utils
)

install(PROGRAMS src/keras_layers/keras_layer_AnchorBoxes.py src/keras_layers/keras_layer_DecodeDetections.py
src/keras_layers/keras_layer_DecodeDetectionsFast.py src/keras_layers/keras_layer_L2Normalization.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/keras_layers
)

install(PROGRAMS src/keras_loss_function/keras_ssd_loss.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/keras_loss_function
)

install(PROGRAMS src/ssd_encoder_decoder/matching_utils.py src/ssd_encoder_decoder/ssd_input_encoder.py src/ssd_encoder_decoder/ssd_output_decoder.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ssd_encoder_decoder
)

install(PROGRAMS src/models/keras_ssd7.py src/models/keras_ssd300.py src/models/keras_ssd512.py  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/models
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
