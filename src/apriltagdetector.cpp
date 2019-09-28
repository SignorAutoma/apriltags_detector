#include <iostream> 
using namespace std;


#include "ros/ros.h"

#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <iostream>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <unistd.h>
#include "AprilTags/common_functions.h"
#include <memory>

#include <nodelet/nodelet.h>

class ContinuousDetector: public nodelet::Nodelet
{
 public:
   ContinuousDetector();
  void onInit();

  void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);

 private:
  std::shared_ptr<TagDetector> tag_detector_;
  bool draw_tag_detections_image_;
  cv_bridge::CvImagePtr cv_image_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  image_transport::Publisher tag_detections_image_publisher_;
  ros::Publisher tag_detections_publisher_;
};


void camCallback(const sensor_msgs::Image::ConstPtr& msg)
{

  	ROS_INFO("Received image with size: %i x %i", msg->width, msg->height);

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat image = cv_ptr -> image;
	cv::Mat gray;


	cv::cvtColor(image, gray, CV_BGR2GRAY); 

			  
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "apriltags_detector");
	
	ros::NodeHandle nh;
	ros::Publisher tags_pub = nh.advertise<std_msgs::String>("tags",100);

	ros::Subscriber camera_image_sub = nh.subscribe("/usb_cam/image_raw", 10, camCallback);
	

	ros::spin();
	
	return 0;
}
	
