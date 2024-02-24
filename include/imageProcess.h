#pragma once

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CompressedImage.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace dummy_ns{

class ImageProcess{
  private:
	ros::NodeHandle& nodeHandle_;

	ros::Publisher pubImg;
	ros::Publisher pubCan;

	ros::Timer canTimer;

	message_filters::Subscriber<sensor_msgs::CompressedImage> leftImgSub;
  	message_filters::Subscriber<sensor_msgs::CompressedImage> rightImgSub;

	/*std::string leftImgTopic;
	std::string rightImgTopic;*/

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
                                                sensor_msgs::CompressedImage> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> sync;
	boost::shared_ptr<sync> syncPtr;

	cv_bridge::CvImageConstPtr convertImage(sensor_msgs::CompressedImage& img, 
		std::string image_encoding = sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImage hConcatImg(const cv_bridge::CvImageConstPtr img1, const cv_bridge::CvImageConstPtr img2);


  public:
	ImageProcess(ros::NodeHandle& nodeHandle);
	virtual ~ImageProcess();

	bool readParameters();
	void syncCallback(const sensor_msgs::CompressedImageConstPtr& leftImgSub, 
					const sensor_msgs::CompressedImageConstPtr& rightImgSub);

	void canPub(const ros::TimerEvent& event);


};
}
