cmake_minimum_required(VERSION 3.0.2)
project(dummy-mate)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
        can_msgs
        sensor_msgs
	message_generation
	cv_bridge
	image_transport
	message_filters
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES 
   std_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dummy-mate
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs can_msgs cv_bridge image_transport message_filters
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 include
)

add_executable(subscribe src/subscribe.cpp src/imageProcess.cpp)
target_link_libraries(subscribe ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
add_dependencies(subscribe dummy-mate_generate_messages_cpp)
