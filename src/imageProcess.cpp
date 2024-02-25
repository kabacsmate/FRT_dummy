#include "imageProcess.h"

namespace dummy_ns{

	ImageProcess::ImageProcess(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
		imgFreq = 0;
		currTime = ros::Time::now();
		prevTime = ros::Time::now();

		// Synchronized subscriptions
		leftImgSub.subscribe(nodeHandle_, "zed_left_img_comp/compressed", 5);
		rightImgSub.subscribe(nodeHandle_, "zed_right_img_comp/compressed", 5);
		syncPtr.reset(new sync(MySyncPolicy(10), leftImgSub, rightImgSub));
		syncPtr->registerCallback(boost::bind(&dummy_ns::ImageProcess::syncCallback, this, _1, _2));

		// Publish CAN messages with the given frequency
		canTimer = nodeHandle_.createTimer(ros::Duration(1/50), &dummy_ns::ImageProcess::canPub, this);
		pubCan = nodeHandle_.advertise<can_msgs::Frame>("sent_messages", 10);
		pubImg = nodeHandle_.advertise<sensor_msgs::Image>("/dummy_mate_img", 5);
	}
	ImageProcess::~ImageProcess(){
		canTimer.stop();
	}

	// Convert a sensor_msgs::CompressedImage type image to a cv_bridge::CvImageConstPtr image pointer
	cv_bridge::CvImageConstPtr ImageProcess::convertImage(sensor_msgs::CompressedImage& img, std::string imageEncoding)
	{
        	cv_bridge::CvImageConstPtr cvptrImg;
        	cvptrImg = cv_bridge::toCvCopy(img, imageEncoding);

		return cvptrImg;
	}

	// Horizontally concatenate 2 CvImageConstPtr image in one cv_bridge::CvImage type
	cv_bridge::CvImage ImageProcess::hConcatImg(const cv_bridge::CvImageConstPtr img1, const cv_bridge::CvImageConstPtr img2){
        	cv::Mat resultImg;
        	cv::hconcat(img1->image, img2->image, resultImg);

        	cv_bridge::CvImage resultImgCV;
        	resultImgCV.header = img1->header;
		resultImgCV.header.stamp = ros::Time::now();
        	resultImgCV.encoding = sensor_msgs::image_encodings::BGR8;
		resultImgCV.image = resultImg;

		return resultImgCV;
	}

	void ImageProcess::syncCallback(const sensor_msgs::CompressedImageConstPtr& leftImgSub, const sensor_msgs::CompressedImageConstPtr& rightImgSub){

		sensor_msgs::CompressedImage leftImg = *leftImgSub;
  		sensor_msgs::CompressedImage rightImg = *rightImgSub;
		currTime = ros::Time::now();

		cv_bridge::CvImageConstPtr cvptrLeft;
                cv_bridge::CvImageConstPtr cvptrRight;
                cv_bridge::CvImage resultImgCV;

                cvptrRight = convertImage(rightImg);
                cvptrLeft = convertImage(leftImg);

                resultImgCV = hConcatImg(cvptrLeft, cvptrRight);
                pubImg.publish(resultImgCV.toImageMsg());

		// Calculating the publishing frequency
		ros::Duration elapsedTime = currTime - prevTime;
		double eTSecDouble = (double)elapsedTime.nsec / 1000000000.0;
		imgFreq = (unsigned int)(1/eTSecDouble);
		prevTime = currTime;
	}

	void ImageProcess::canPub(const ros::TimerEvent& event){
        	can_msgs::Frame canFrame;
        	canFrame.id = 0x414;
        	canFrame.is_rtr = canFrame.is_extended = canFrame.is_error = false;
        	canFrame.dlc = 8;
        	canFrame.data[0] = imgFreq;
        	for (int i = 1 ; i < 8 ; i++){
                	canFrame.data[i] = 0;
        	}

        	pubCan.publish(canFrame);
	}

}
