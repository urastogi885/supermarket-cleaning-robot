cmake_minimum_required(VERSION 2.8.3)
project(supermarket_cleaning_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(OpenCV REQUIRED)

find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	sensor_msgs
  geometry_msgs
  move_base_msgs
	cv_bridge
	image_transport
)

## Declare a catkin package
catkin_package()

## Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add executables
add_executable(object_collection src/turtlebot.cpp src/obstacle_avoidance.cpp src/main.cpp)

add_executable(detector src/object_detection.cpp src/detector.cpp)

## Add target link libraries
target_link_libraries(object_collection ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Enable catkin test and add files for the same
if(CATKIN_ENABLE_TESTING)
  set(CMAKE_BUILD_TYPE Debug)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
   test_project_x_robot
   test/project_x_test.launch
   test/main.cpp
   test/obstacle_avoidance_test.cpp
   test/turtlebot_test.cpp
   src/obstacle_avoidance.cpp
   src/turtlebot.cpp)

  target_link_libraries(
   test_project_x_robot 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS})

  add_dependencies(
  	test_project_x_robot
    object_collection
    detector
    ${catkin_EXPORTED_TARGETS})
endif()