/*!
 * \hazard_detection.cpp
  * \brief Node for finding hazards in disparity images
 *
 * HazardDetection creates a ROS node that uses the disparity images to detect obstacles
 * and send the corresponding point clouds
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Derek Ross, WVU - 
 * \author Dr. Guilherme Pereira, WVU - 
 * \date May 10, 2021
 */

#include "src2_object_detection/hazard_detection.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//#define SHOWIMG

void HazardDetection::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
}


HazardDetection::HazardDetection(ros::NodeHandle & nh)
: nh_(nh)
{

  #ifdef SHOWIMG  
    cv::namedWindow("originall");
    cv::startWindowThread(); 
    cv::setMouseCallback("originall", CallBackFunc, this);
  #endif  

  // Subscribers  
  subOdometry = nh_.subscribe("localization/odometry/sensor_fusion", 1, &HazardDetection::odometryCallback, this);
  subJointStates = nh_.subscribe("joint_states", 1, &HazardDetection::jointStateCallback, this);
  subDisparity = nh_.subscribe("disparity", 1, &HazardDetection::disparityCallback, this);
  
  currSensorYaw_=0.0;
  currSensorPitch_=0.0;
  
}

HazardDetection::~HazardDetection()
{
  cv::destroyAllWindows();
}

void HazardDetection::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (firstOdom_ == false)
    {
        firstOdom_ = true;
    }
    currOdom_ = *msg;
}

void HazardDetection::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  int sensor_bar_yaw_joint_idx;
  int sensor_bar_pitch_joint_idx;

  // loop joint states
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "sensor_bar_yaw_joint") {
      sensor_bar_yaw_joint_idx = i;
    }
    if (msg->name[i] == "sensor_bar_pitch_joint") {
      sensor_bar_yaw_joint_idx = i;
    }
  }

 currSensorYaw_ = msg->position[sensor_bar_yaw_joint_idx];  
 currSensorPitch_ = msg->position[sensor_bar_pitch_joint_idx]; 
}

void HazardDetection::disparityCallback(const stereo_msgs::DisparityImagePtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
    #ifdef SHOWIMG     
        cv::imshow("original", cv_ptr->image);
    #endif    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  disparity_image_ = cv_ptr->image;
}
      
      
void HazardDetection::ComputeHazards()
{    
  cv::Mat hsv_disparity;
  cv::cvtColor(disparity_image_, hsv_disparity, CV_BGR2HSV);
  // cv::imshow("hsv_imagel", hsv_imagel);
} 


/*!
 * \brief Creates and runs the HazardDetection node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hazard_detection");
  ros::NodeHandle nh("");
  ros::Rate rate(50);

  ROS_INFO("Hazard Detection Node initializing...");
  HazardDetection hazard_detection(nh);

  while(ros::ok()) 
  {
		ros::spinOnce();
		rate.sleep();
	}
  return 0;
}
