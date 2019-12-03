cmake_minimum_required(VERSION 2.8.3)
project(supermarket_cleaning_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(OpenCV 2 REQUIRED)

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
add_executable(supermarket_cleaning_robot src/turtlebot.cpp src/obstacle_avoidance.cpp)

## Add target link libraries
## target_link_libraries(supermarket_cleaning_robot ${catkin_LIBRARIES}${OpenCV_INCLUDE_DIRS})

## Enable catkin test and add files for the same
if(CATKIN_ENABLE_TESTING)
  set(CMAKE_BUILD_TYPE Debug)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
   test_project_x_robot
   test/main.cpp 
   test/obstacle_avoidance_test.cpp 
   test/turtlebot_test.cpp 
   src/turtlebot.cpp 
   src/obstacle_avoidance.cpp)

  target_link_libraries(
   test_supermarket_cleaning_robot 
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS})

  add_dependencies(
    supermarket_cleaning_robot
    test_project_x_robot
    ${catkin_EXPORTED_TARGETS})
endif()