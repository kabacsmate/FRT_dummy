#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <thread>
#include "imageProcess.h"

// The images from the image topics
sensor_msgs::CompressedImage leftImg;
sensor_msgs::CompressedImage rightImg;
// Check if the images are available
bool isRightImg = false;
bool isLeftImg = false;

// Save the synchronized images
void syncCallback(const sensor_msgs::CompressedImageConstPtr& leftImgSub, const sensor_msgs::CompressedImageConstPtr& rightImgSub){
  leftImg = *leftImgSub;
  rightImg = *rightImgSub;
  isLeftImg = isRightImg = true;

  ROS_INFO("The left and right images have arrived! %d %d ", leftImg.header.seq, rightImg.header.seq);
}

// Own thread for the 50Hz CAN communication
void canThread(){
	ros::NodeHandle n;
	ros::Publisher pubCan = n.advertise<can_msgs::Frame>("sent_messages", 10);
	unsigned int frequencyRate = 50; // The frequency of the messages
	can_msgs::Frame canFrame;

	canFrame.id = 0x414;
  	canFrame.is_rtr = canFrame.is_extended = canFrame.is_error = false;
  	canFrame.dlc = 8;
  	canFrame.data[0] = frequencyRate;
	for (int i = 1 ; i < 8 ; i++){
		canFrame.data[i] = 0;
	}

  	ros::Rate rate(frequencyRate);

	while (ros::ok()){
		pubCan.publish(canFrame);
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imgListener");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::Image>("dummy_img_mate", 10);

  // Variables for the synchronization
  message_filters::Subscriber<sensor_msgs::CompressedImage> leftImgSub(n, "zed_left_img_comp/compressed", 1);
  message_filters::Subscriber<sensor_msgs::CompressedImage> rightImgSub(n, "zed_right_img_comp/compressed", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, 
						sensor_msgs::CompressedImage> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), leftImgSub, rightImgSub);
  sync.registerCallback(boost::bind(&syncCallback, _1, _2));

  // Define the thread for the CAN communication
  std::thread thread1(canThread);

  // Concatenate the images periodically
  while(ros::ok()){
	if(isRightImg && isLeftImg)
        {
  		cv_bridge::CvImageConstPtr cvptrLeft;
  		cv_bridge::CvImageConstPtr cvptrRight;
		cv_bridge::CvImage resultImgCV;

		cvptrRight = convertImage(rightImg);
		cvptrLeft = convertImage(leftImg);

		resultImgCV = hConcatImg(cvptrLeft, cvptrRight);
		pub.publish(resultImgCV.toImageMsg());

		isRightImg = isLeftImg = false;
  	}
	ros::spinOnce();
  }
  ros::spin();

  return 0;
}
