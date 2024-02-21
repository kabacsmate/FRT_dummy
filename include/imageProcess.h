#include "sensor_msgs/CompressedImage.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>



cv_bridge::CvImageConstPtr convertImage(sensor_msgs::CompressedImage& img, 
		std::string image_encoding = sensor_msgs::image_encodings::BGR8);
cv_bridge::CvImage hConcatImg(const cv_bridge::CvImageConstPtr img1, const cv_bridge::CvImageConstPtr img2);
