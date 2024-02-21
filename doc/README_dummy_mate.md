# dummy-mate 

## Description
The dummy_img_mate node can concatenate 2 images horizontally and publish CAN messages with different frequency.

## Project structure
The file structure of the project, to help someone to find the specific file they are looking for.

TODO

```
.
├── cfg
│   ├── config.yaml
├── doc                                 # Documentation of the package
│   ├── documentation.pdf                    
├── include                             # Public parts of the package (API)
│   ├── header.h                        # Utility functions
├── launch
│   ├── launch.launch                   # Launch file to use in the AS pipeline
│   └── simLaunch.launch                # Launch file to use with simulator
├── src                                 # Private parts of the package
│   ├── code.cpp                        # Main implementation file
│   ├── rosIF.cpp                       # ROS intereface implementation
├── CMakeLists.txt                      # Main CMake file
├── package.xml
└── README.md
```

## Dependencies
Here you can assume the user has everything that is needed to run the AS (ROS, catkin etc.).

- [roscpp](https://github.com/ros/ros_comm)
- [rospy](https://github.com/ros/ros_comm)
- [std_msgs](https://github.com/ros/std_msgs)
- [sensor_msgs](https://github.com/ros/common_msgs)
- [can_msgs](https://github.com/BME-FRT/roscan_open)
- [message_runtime](https://github.com/ros/message_runtime)
- [cv_bridge](https://github.com/ros-perception/vision_opencv)
- [image_transport](https://github.com/ros-perception/image_common)
- [message_filters](https://github.com/ros/ros_comm)


## Usage

The package can be launched with the following command:

roslaunch dummy-mate dummy_mate.launch

It contains the single_lap.bag file, you may need to modify its path!

## Configuration

No config file.

## Topics
List all the topics the package is subscribed to or publishes with its type
### Subscribed to
- [sensor_msgs/CompressedImage] /zed_right_img_comp/compressed : The right ZED camera image for the dummy project.
- [sensor_msgs/CompressedImage] /[zed_left_img_comp/compressed] : The left ZED camera image for the dummy project.
### Publishes
- [sensor_msgs/Image] /dummy_img_mate : Contains the horizontally concatenated images.
- [can_msgs/Frame] /sent_messages : This is a CAN frame what is periodically published and it’s frequency can be changed.

## Miscellaneous 