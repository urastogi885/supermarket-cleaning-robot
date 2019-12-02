cmake_minimum_required(VERSION 2.8.3)
project(supermarket_cleaning_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	sensor_msgs
  	geometry_msgs
  	move_base_msgs
	cv_bridge
	image_transport
)

find_package(OpenCV 2 REQUIRED)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS
	CATKIN_DEPENDS 	roscpp
                   	sensor_msgs
                   	std_msgs
                   	cv_bridge
                   	image_transport
	)

## Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add executables
## add_executable(supermarket_cleaning_robot SOURCE_FILE_NAME)

## Add target link libraries
## target_link_libraries(supermarket_cleaning_robot ${catkin_LIBRARIES}${OpenCV_INCLUDE_DIRS})

## Enable catkin test and add files for the same
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
