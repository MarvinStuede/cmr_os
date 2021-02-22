# **cmr_os**: Main packages for Campus Maschinenbau Robot (Sobi)

This repository contains the basic packages to use the robot. This includes launch files to start sensors, utility functions a a gazebo simulation.
Each package contains a separate README with instructions.

### Overview

##### cmr_api
This package contains Catkin libraries to access basic robot functions such as speech, arm or ear movement from C++ code.

##### cmr_bt_generic
High level control of the robot is realized by [Behavior Trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)) with the [BehaviorTree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP). This packages contains some general, system-wide usable nodes and conditions.

##### cmr_description
URDF description

##### cmr_driver
 Launch- and configuration files to start the drivers for all sensors and actuators

##### cmr_gazebo
Contains some example worlds and a launch file to start a simple Gazebo simulation with basic sensors and a moveable robot

##### cmr_os
Contains some general utility functions

### Installing
See general instructions [here](https://marvinstuede.github.io/Sobi/software/)

If `catkin build` fails for some gazebo plugins you need to install libignition-math2-dev via apt
```
sudo apt install libignition-math2-dev
```
