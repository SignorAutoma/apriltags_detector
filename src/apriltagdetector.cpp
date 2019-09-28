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

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"


/* #include "../include/AprilTags/Tag16h5.h"
#include "../include/AprilTags/Tag25h7.h"
#include "../include/AprilTags/Tag25h9.h"
#include "../include/AprilTags/Tag36h9.h"
#include "../include/AprilTags/Tag36h11.h" */




#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;


inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}



//rotation matrix to euler
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

class tag {
	
	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;
	
	int width;
	int height;
	double tagSize; //april tag size in square frame
	double fx; // camera focal length in pixels
	double fy;
	double px; // camera principal point
	double py;
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
	
    void setupByTag() {
		m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11)

	}
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

	void detection_print(AprilTags::TagDetection& detection) const {
		cout << " Id: " << detection.id << " (Distance: " << detection.hammingDistance << ")";
		
		//Get relative position of the tag 
		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
												translation, rotation);

		Eigen::Matrix3d F;
		F <<
		1, 0,  0,
		0,  -1,  0,
		0,  0,  1;
		Eigen::Matrix3d fixed_rot = F * rotation;
		double yaw, pitch, roll;

		wRo_to_euler(fixed_rot, yaw, pitch, roll);
		cout << "  distance=" << translation.norm()
				<< "m, x=" << translation(0)
				<< ", y=" << translation(1)
				<< ", z=" << translation(2)
				<< ", yaw=" << yaw
				<< ", pitch=" << pitch
				<< ", roll=" << roll
				<< endl;
	}

	void imageConsumer() {
	

	}

} 

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


	cv::cvtColor(image, gray, CV_BGR2GRAY); //REALLY NEEDED?
	vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(gray);

	cout << detections.size() << " tags detected:" << endl;
		for (int i=0; i<detections.size(); i++) {
		print_detection(detections[i]);
	}
			
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
	
