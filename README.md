# Project X
[![Build Status](https://travis-ci.org/urastogi885/Supermarket-Cleaning-Robot.svg?branch=master)](https://travis-ci.org/urastogi885/Supermarket-Cleaning-Robot)
[![Coverage Status](https://coveralls.io/repos/github/urastogi885/Supermarket-Cleaning-Robot/badge.svg?branch=master)](https://coveralls.io/github/urastogi885/Supermarket-Cleaning-Robot?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/urastogi885/Supermarket-Cleaning-Robot/blob/master/LICENSE)
---

## Overview

According to a study done in Morrisville, North Carolina, the Walmart Supercenter located in the town
receives about 10,000 people per day. Unquestionably, the actual foot traffic depends on a variety of
factors, but we cannot disregard that supermarkets are one of the busiest places in a town. The more the
number of people, the more likely it is for the store to become dirty. It always begets frustration among
workers to maintain the store at its most pristine level. Our supermarket cleaning robot can remove the
stress of cleanliness by performing the tasks of an employee.

Currently, most of the robots are only capable of executing a single task. It turns out to be expensive for
a store owner to buy a robot that can perform a single task. We propose to develop a robot that can
perform various maintenance tasks. The robot will be able to maintain cleanliness as well as make
supermarkets autonomous. The robot will able to clean aisles, stack up empty rows, and collect fallen
items.

For prototyping, we are focusing on only one task that is identifying and collecting the items using the
robot. The robot will roam in a supermarket like environment in Gazebo and identify the type of items
that it needs to collect. It identifies the item using a camera, mounted on its base, and moves towards the
fallen item. Here, we are considering objects such as food, soft drinks cans and it is assumed that the robot
will already know the type of item that it needs to pick. As the robot reaches the location of the item and
touches it, the item will vanish depicting that the item is collected using a suction cup. The robot will
traverse randomly in the supermarket and keep on collecting a can. We are focusing on the detection of
cans using the OpenCV to improve the processing of the detection feature. In addition to this, the robot
has an obstacle avoidance feature that is used to prevent the robot from colliding from obstacles such as
humans, uninteresting items and walls/shelves.

<p align="center">
<img src="https://github.com/urastogi885/Supermarket-Cleaning-Robot/blob/master/data/readme_images/initial_proposal_setup.png">
<b>Figure 1 - Robot approaching towards the cans lying on the ground to collect them</b>
</p>


## Team Members

- [Umang Rastogi](https://github.com/urastogi885) - Pursuing masters in Robotics at University of Maryland | B.Tech in Electronics & Communication Engineering
- [Naman Gupta](https://github.com/namangupta98) - Grad Student at University of Maryland, pursuing M.Eng. in Robotics.

## AIP and Sprint Documents

- Click on this [*link*](https://docs.google.com/spreadsheets/d/1k6e7rM7TTvE5w2fQ_wuSDY_giNWaVuCHeImB6D53lT4/edit?usp=sharing)
to access our AIP Google Sheet.
- Click on this [*link*](https://docs.google.com/document/d/1iQZUstgoCCvtSvlcv1_xpxGW6ntUbkOpcgMuvrSP_ms/edit?usp=sharing)
to access our Sprint notes document.

## Accessing the UML Diagrams

- Open the *UML* directory of the project.
- Access UML diagrams from the *initial* folder located within *UML* sub-directory.

## API Documentations

- [Gazebo Population Tag](http://gazebosim.org/tutorials?tut=model_population&cat=build_world)
- [cv_bridge](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
- [Template Matching](https://docs.opencv.org/master/de/da9/tutorial_template_matching.html)

## Dependencies

- Ubuntu 16.04
- ROS Kinetic
- Gazebo
- Turtlebot Packages

## Install Dependences

- This project was developed using ROS Kinetic.
- It is highly recommended that ROS Kinetic is properly installed on your system before the use of this project.
- Follow the instructions on the [*ROS kinetic install tutorial page*](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  to install ***Full-Desktop Version*** of ROS Kinetic.
- The full-version would help you install *Gazebo* as well. If you have ROS Kinetic pre-installed on your machine, use
  the following [*link*](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) to just install *Gazebo* on your
  machine.
- Ensure successful installation by running *Gazebo* via your terminal window:
```shell script
gazebo
```
- An empty window of *Gazebo Simulator* should be launched.
- Make sure that turtlebot packages have been installed on your machine using the following commands:
```shell script
roslaunch turtlebot_gazebo turtlebot_world.launch
``` 
- A window of *Gazebo Simulator* with various items and a turtlebot should be launched.
- If an error pops up upo launching the turtlebot world, then install the necessary turtlebot packages:
```shell script
sudo apt install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
- Create your ROS workspace by following instructions on the [*create ROS workspace tutortial page*](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
   
## Known Bugs and Issues

This project is under-development. Currently, we are facing build issues. Sorry for the inconvenience.

## Build

- ***Ignore this section*** as nothing to be built has been added yet.
- Even if you run the following, it will not impact your existing workspace.
- Switch to your *src* sub-directory of your ROS workspace to clone this repository.
```shell script
<ROS Workspace>/src
```
- Run the following commands to clone and build this project:
```shell script
git clone --recursive https://github.com/urastogi885/obstacle_avoidance_simulation
git checkout Phase3
cd ..
catkin_make
```

## Test

Close and terminate everything including rosmaster. In a new terminal, switch to the ROS workspace and build the tests. Type

```
cd catkin_ws
source devel/setup.bash
catkin_make run_tests_test_project_x_robot
```

## Run

Now, we use launch file to run. In a new terminal, type

```
cd catkin_ws
source devel/setup.bash
roslaunch supermarket_cleaning_robot object_collection.launch
```

## Doxygen

To install doxygen run the following command:

```
sudo apt-get install doxygen
```
Now from the cloned directory run:

```
doxygen doxygen
```

Generated doxygen files are in html format and you can find them in ./docs folder. With the following command

```
firefox docs/html/index.html
```

## Demo

We will update in the next phase by next week.