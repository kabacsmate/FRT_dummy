#include <ros/ros.h>
#include "imageProcess.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imgListener");
  ros::NodeHandle n;

  dummy_ns::ImageProcess imgProc(n);
  //imgProc.canPub();

  ros::spin();

  return 0;
}
