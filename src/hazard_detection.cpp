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
: nh_(nh), sync(MySyncPolicy(10), left_image_sub, left_info_sub, right_image_sub, right_info_sub)
{
  ximg=0;
  yimg=0;
#ifdef SHOWIMG  
  cv::namedWindow("originall");
  cv::startWindowThread(); 
  cv::setMouseCallback("originall", CallBackFunc, this);
#endif  


  // Subscribers  
  subOdometry = nh_.subscribe("localization/odometry/sensor_fusion", 1, &HazardDetection::odometryCallback, this);
  subJointStates = nh_.subscribe("joint_states", 1, &HazardDetection::jointStateCallback, this);
  
  right_image_sub.subscribe(nh_,"camera/right/image_raw", 1);
  left_image_sub.subscribe(nh_,"camera/left/image_raw", 1);
  right_info_sub.subscribe(nh_,"camera/right/camera_info", 1);
  left_info_sub.subscribe(nh_,"camera/left/camera_info", 1);
  
  sync.registerCallback(boost::bind(&HazardDetection::imageCallback,this, _1, _2, _3, _4));
  
  iLowH_ = 0;
  iHighH_ = 5;

  iLowS_ = 130;
  iHighS_ = 250;

  iLowV_ = 100;
  iHighV_ = 255;
  
  currSensorYaw_=0.0;
  direction_ = 1;
  
}

HazardDetection::~HazardDetection()
{
  cv::destroyAllWindows();
}


bool HazardDetection::compareKeypoints(const cv::KeyPoint &k1, const cv::KeyPoint &k2)
{
	if (k1.size > k2.size) return true;
  	else return false;
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

void HazardDetection::imageCallback(const sensor_msgs::ImageConstPtr& msgl, const sensor_msgs::CameraInfoConstPtr& info_msgl, const sensor_msgs::ImageConstPtr& msgr, const sensor_msgs::CameraInfoConstPtr& info_msgr)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgl, sensor_msgs::image_encodings::BGR8);
#ifdef SHOWIMG     
    cv::imshow("originall", cv_ptr->image);
#endif    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  raw_imagel_ = cv_ptr->image;
  info_msgl_ = *info_msgl;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgr, sensor_msgs::image_encodings::BGR8);
#ifdef SHOWIMG         
    cv::imshow("original1", cv_ptr->image);
#endif    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  raw_imager_ = cv_ptr->image;
  info_msgr_ = *info_msgr;
    
  ////////////////////////////////////////
  
}
      
      
void HazardDetection::ComputeHazards()
{    
  cv::Mat hsv_imagel;
  cv::cvtColor(raw_imagel_, hsv_imagel, CV_BGR2HSV);
 // cv::imshow("hsv_imagel", hsv_imagel);
  
  cv::Mat hsv_imager;
  cv::cvtColor(raw_imager_, hsv_imager, CV_BGR2HSV);
 // cv::imshow("hsv_imager", hsv_imager);
   
  //cv::Vec3b hsvPixel = hsv_imagel.at<cv::Vec3b>(280,484);
  //ROS_INFO("%d %d %d", hsvPixel.val[0], hsvPixel.val[1], hsvPixel.val[2] );
  
  cv::Mat imgThresholdedl;
  cv::inRange(hsv_imagel, cv::Scalar(iLowH_, iLowS_, iLowV_), cv::Scalar(iHighH_, iHighS_, iHighV_), imgThresholdedl); 
  cv::dilate(imgThresholdedl,imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(imgThresholdedl,imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 3);
  
  cv::Mat imgThresholdedr;
  cv::inRange(hsv_imager, cv::Scalar(iLowH_, iLowS_, iLowV_), cv::Scalar(iHighH_, iHighS_, iHighV_), imgThresholdedr); 
  cv::dilate(imgThresholdedr,imgThresholdedr, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(imgThresholdedr,imgThresholdedr, cv::Mat(), cv::Point(-1, -1), 3);
  
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Filter by Area.
  params.filterByColor = false;
  params.blobColor = 255;

  // Change thresholds
  //params.minThreshold = 10;
  //params.maxThreshold = 200;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 5000;
  params.maxArea = 2000000;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.1;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;
  
  // SimpleBlobDetector::create creates a smart pointer.
  // Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  // Detect blobs.
  std::vector<cv::KeyPoint> keypointsl, keypointsr;
  detector->detect( imgThresholdedl, keypointsl);
  detector->detect( imgThresholdedr, keypointsr);

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
#ifdef SHOWIMG  
  cv::Mat im_with_keypointsl, im_with_keypointsr; 
  cv::drawKeypoints( imgThresholdedl, keypointsl, im_with_keypointsl, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  cv::drawKeypoints( imgThresholdedr, keypointsr, im_with_keypointsr, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
#endif
  
  if ((keypointsl.size() > 0) && (keypointsr.size() > 0))
  {
 	// Sort the keypoints in order of area
 	std::sort(keypointsl.begin(), keypointsl.end(), compareKeypoints);
 	std::sort(keypointsr.begin(), keypointsr.end(), compareKeypoints);
 	
 	/*for (int i=0;i<keypointsl.size();i++){
 		ROS_INFO("Left: Area %f", keypointsl[i].size);
 	}
 	for (int i=0;i<keypointsr.size();i++){
 		ROS_INFO("Righ: Area %f", keypointsr[i].size);
 	}
 	ROS_INFO("____");*/
 	
 	
 	int i=0; // Get the larger keypoints in each camera
 	
 	// check if is the same object 
 	if (fabs(keypointsl[i].size - keypointsr[i].size)<20)
 	{
 		double disparity = keypointsl[i].pt.x - keypointsr[i].pt.x;
 		double offset = keypointsl[i].pt.y - keypointsr[i].pt.y;
 	
 		if(disparity > 5 && disparity < 100)
		{
			//check epipolar constraint
     			 if(offset < 5 && offset > -5)
			{
				double cx = (double) info_msgl_.P[2];
				double cy = (double) info_msgl_.P[6];
				double sx = (double) info_msgl_.P[0];
				double sy = (double) info_msgl_.P[5];
				double bl = (double) (-(double)info_msgr_.P[3]/info_msgr_.P[0]);

				z_ = sx/disparity*bl;
				x_ = (keypointsl[i].pt.x-cx)/sx*z_;
				y_ = (keypointsl[i].pt.y-cy)/sy*z_;
			
				ROS_INFO("(%f,%f,%f)", x_, y_, z_);
				target_.point.x = x_;
        			target_.point.y = y_;
        			target_.point.z = z_;
        			pubTarget.publish(target_);
			}
		}
	}
  } 

  
  
#ifdef SHOWIMG 
  // Show blobs
  imshow("blobsl", im_with_keypointsl);
  imshow("blobsr", im_with_keypointsr);
#endif  
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
  ros::init(argc, argv, "find_rover");
  ros::NodeHandle nh("");
  ros::Rate rate(50);

  ROS_INFO("Find Rover Node initializing...");
  HazardDetection find_rover(nh);

  while(ros::ok()) 
  {
		ros::spinOnce();
		rate.sleep();
	}
  return 0;
}
