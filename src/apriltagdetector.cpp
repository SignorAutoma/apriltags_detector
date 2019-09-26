#include <iostream> 
using namespace std;


#include "ros/ros.h"

#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <fstream>
#include <iostream>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <unistd.h>




/* #include "../include/AprilTags/Tag16h5.h"
#include "../include/AprilTags/Tag25h7.h"
#include "../include/AprilTags/Tag25h9.h"
#include "../include/AprilTags/Tag36h9.h"
#include "../include/AprilTags/Tag36h11.h" */

/* 
class tag {
	
	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;
	
	int width;
	int height;
	double tagSize; //april tag size in square frame
	double fx; // camera focal length in pixels
	double fy;
	double px; // camera principal point
	double py
	cv:VideoCApture cap;
	
public:

	tag() : 
	
		m_tagDetector(NULL),
		m_tagCodes(AprilTags::tagCodes36h11)
		
		width(640),
		height(480),
		tagSize(0.166),
		fx(600),
		fy(600),
		px(m_width/2),
		py(m_height/2)
		
    {}
    
    void playCam() {
		string video_str = "/dev/video0";
		
		m_cap = cv::VideoCapture(0);
		if(!m_cap.isOpened()) {
			cerr << "ERROR: Can't find video device " << 0 << "\n";
			exit(1);
		}
		
		m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
		
		cout << "Camera opened" << endl;
		cout << "Actual resolution: "
			 << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
			 << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

	  }

} */

void camCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Received image with size: %i x %i", msg->width, msg->height);
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "apriltags_detector");
	
	ros::NodeHandle nh;
	ros::Publisher tags_pub = nh.advertise<std_msgs::String>("tags",100);

	ros::Subscriber camera_image_sub = nh.subscribe("/usb_cam/image_raw", 10, camCallback);
	
	/*
	Tag apriltag;
	apriltag.playCam()*/
/* 
	ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg); */

	ros::spin();
	
	return 0;
}
	
