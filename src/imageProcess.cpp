#include "imageProcess.h"

namespace dummy_ns{

	ImageProcess::ImageProcess(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
		ROS_INFO("In the Constructor!");
		leftImgSub.subscribe(nodeHandle_, "zed_left_img_comp/compressed", 5);
		rightImgSub.subscribe(nodeHandle_, "zed_right_img_comp/compressed", 5);
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage,
                                                sensor_msgs::CompressedImage> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), leftImgSub, rightImgSub);
		sync.registerCallback(boost::bind(&dummy_ns::ImageProcess::syncCallback, this, _1, _2));
		pubImg = nodeHandle_.advertise<sensor_msgs::Image>("dummy_img_mate", 10); //<-- here (this)
		ROS_INFO("Constructor finished!");
	}
	ImageProcess::~ImageProcess(){}

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
        	resultImgCV.encoding = sensor_msgs::image_encodings::BGR8;
		resultImgCV.image = resultImg;

		return resultImgCV;
	}

	void ImageProcess::syncCallback(const sensor_msgs::CompressedImageConstPtr& leftImgSub, const sensor_msgs::CompressedImageConstPtr& rightImgSub){
  		ROS_INFO("In the Callback");
		sensor_msgs::CompressedImage leftImg = *leftImgSub;
  		sensor_msgs::CompressedImage rightImg = *rightImgSub;
  		//isLeftImg = isRightImg = true;

		cv_bridge::CvImageConstPtr cvptrLeft;
                cv_bridge::CvImageConstPtr cvptrRight;
                cv_bridge::CvImage resultImgCV;

                cvptrRight = convertImage(rightImg);
                cvptrLeft = convertImage(leftImg);

                resultImgCV = hConcatImg(cvptrLeft, cvptrRight);
                pubImg.publish(resultImgCV.toImageMsg());

  		ROS_INFO("The left and right images have arrived! %d %d ", leftImg.header.seq, rightImg.header.seq);
	}

	void ImageProcess::canPub(){
        	pubCan = nodeHandle_.advertise<can_msgs::Frame>("sent_messages", 10);
        	unsigned int frequencyRate = 50; // The frequency of the messages
        	can_msgs::Frame canFrame;

        	canFrame.id = 0x414;
        	canFrame.is_rtr = canFrame.is_extended = canFrame.is_error = false;
        	canFrame.dlc = 8;
        	canFrame.data[0] = frequencyRate; //This should be the frequency of the images publishing
        	for (int i = 1 ; i < 8 ; i++){
                	canFrame.data[i] = 0;
        	}

        	ros::Rate rate(frequencyRate);

        	while (ros::ok()){
                	pubCan.publish(canFrame);
                	rate.sleep();
        	}
	}

}
