#include <iostream> 
using namespace std;


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

#include "apriltags_detector/AprilTag.h" // rosmsg
#include "apriltags_detector/AprilTagList.h" 

#include <fstream>
#include <iostream>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <unistd.h>
#include <memory>


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

class tagToDetect {
	
	AprilTags::TagDetector* tagDetector;
	AprilTags::TagCodes tag_codes;

	
	int width;
	int height;
	
	double tagSize; //april tag size in square frame
	double fx; // camera focal length in pixels
	double fy;
	double px; // camera principal point
	double py;

	image_transport::Subscriber subscriber;
	image_transport::ImageTransport it_;
  	ros::Publisher tag_publisher;

	ros::NodeHandle nh;

	
public:
	tagToDetect() : 
	    it_(nh), 
		tagDetector(NULL),
		tag_codes(AprilTags::tagCodes36h11),
		width(640),
		height(480),
		tagSize(0.166),
		fx(600),
		fy(600),
		px(width/2),
		py(height/2)
	{	
		cout << "------------ DEBUG TAG CREATE--------------"<< endl;
    	subscriber = it_.subscribe("/camera/image_raw", 1, &tagToDetect::camCallback, this);
		tag_publisher = nh.advertise<apriltags_detector::AprilTagList>("/apriltags_detector",100);

		ros::NodeHandle private_nh("~"); //resolve namespace issue with a private namespace -> mutliistances allowed
		private_nh.param<double>("fx", fx, 600.0);
    	private_nh.param<double>("tagSizeCm", tagSize, 16.6);
    	
		fy = fx; // boh
		tagSize = tagSize /100;

		tagDetector = new AprilTags::TagDetector(tag_codes);
		cout << "got focal length " << fx << endl; //dbug
    	cout << "got tag size " << tagSize << endl;

	}

	apriltags_detector::AprilTag detection_print(AprilTags::TagDetection& detection, int width, int height) {
		
		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(tagSize, fx, fy, px, py,
												translation, rotation);

		Eigen::Matrix3d F;
		F <<
		1, 0,  0,
		0,  -1,  0,
		0,  0,  1;

		Eigen::Matrix3d fixed_rot = F * rotation;
		double yaw, pitch, roll;

		wRo_to_euler(fixed_rot, yaw, pitch, roll);
		//debug print
		cout << "  distance=" << translation.norm()
				<< "m, x=" << translation(0)
				<< ", y=" << translation(1)
				<< ", z=" << translation(2)
				<< ", yaw=" << yaw
				<< ", pitch=" << pitch
				<< ", roll=" << roll
				<< endl;
		//send msgs
		apriltags_detector::AprilTag msg;

		msg.id = detection.id;
		msg.hamming_distance = detection.hammingDistance;
		msg.distance = translation.norm() * 100; //* 100 is due to cm to m
		msg.x = translation(0) * 100;
		msg.y = translation(1) * 100;
		msg.z = translation(2) * 100;
		msg.yaw = yaw;
    	msg.pitch = pitch;
    	msg.roll = roll;

		return msg;
	}

	void camCallback(const sensor_msgs::ImageConstPtr& img_msg)
	{

		ROS_INFO("Received image with size: %i x %i", img_msg->width, img_msg->height);

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat image = cv_ptr -> image;
		cv::Mat image_gray;

		cv::cvtColor(image, image_gray, CV_BGR2GRAY); 

	    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);
		vector<apriltags_detector::AprilTag> msgs;
			
		
		cout << detections.size() << " tags detected:" << endl;	
		for (int i=0; i<detections.size(); i++) {
			msgs.push_back(detection_print(detections[i], cv_ptr -> image.cols, cv_ptr -> image.rows));
		}
		
		if(detections.size() > 0) {
			apriltags_detector::AprilTagList msg_list;
			msg_list.april_tags = msgs;
			tag_publisher.publish(msg_list);
		}
	}

};


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "apriltags_detector");
	cout << "-----------DEBUG MAIN--------------"<< endl;

	tagToDetect newTagDetection;
	ros::spin();
	
	return 0;
}
	
