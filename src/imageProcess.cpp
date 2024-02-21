#include "imageProcess.h"

// Convert a sensor_msgs::CompressedImage type image to a cv_bridge::CvImageConstPtr image pointer
cv_bridge::CvImageConstPtr convertImage(sensor_msgs::CompressedImage& img, std::string imageEncoding)
{
        cv_bridge::CvImageConstPtr cvptrImg;
        cvptrImg = cv_bridge::toCvCopy(img, imageEncoding);

	return cvptrImg;
}

// Horizontally concatenate 2 CvImageConstPtr image in one cv_bridge::CvImage type
cv_bridge::CvImage hConcatImg(const cv_bridge::CvImageConstPtr img1, const cv_bridge::CvImageConstPtr img2){
        cv::Mat resultImg;
        cv::hconcat(img1->image, img2->image, resultImg);

        cv_bridge::CvImage resultImgCV;
        resultImgCV.header = img1->header;
        resultImgCV.encoding = sensor_msgs::image_encodings::BGR8;
	resultImgCV.image = resultImg;

	return resultImgCV;
}
