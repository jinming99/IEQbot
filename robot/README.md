# Autonomous Robot Navigation (ata_navigation)

## Introduction

This package, named *ata_navigation*, contains the navigation and remote control code to run a Turtlebot 2 with ROS. It is organized as follows:

- */src/patrol_bot.cpp:* C++ implementation of autonomous robot navigation

- */scripts/manager.py:* python scripts for remote control using dweet.io

- */launch:* launch files to start ROS.
  - *prep.launch:* bring up Turtlebot and launch basic ROS nodes. Also load the map as specified by the *map_file* parameter.

  - *CBE.launch:* an example launch file that follows prep.launch. This file specifies a list of goal positions and starts the actual navigation.

- */maps:* maps used for navigation

- */log:* default location to store logs during navigation

## Running this package
This package is compatible with Turtlebot 2 and ROS indigo. Before proceeding, configure the Turtlebot ([tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo)) and add this package to ROS_PACKAGE_PATH.

To run this package, first open a terminal and run the following:

`roslaunch ata_navigation prep.launch`

This will bring up the Turtlebot, launch basic ROS nodes, and load the map. To start the actual navigation, run a second launch file that contains the goal positions(see *launch/CBE.launch* for example):

`roslaunch ata_navigation CBE.launch`
