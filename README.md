# Gazebo-simulator

Gazebo is an Open Source simulator very useful for control and path planning applicationes, such as autonomous driving among others. 

## Pre requisites

If you have reached this website, you may want to run this project on your computer and try it out yourself. Well, first of all, you and your computer need to meet some conditions. Some of the following sections are a must if you want to run the simulator on your computer. If you are already done with this part, you can skip it and go ahead with the following.

### Software

* Ubuntu 18.04
* Having installed ROS (melodic version). If you dont have set up ROS on your computer, visit the wiki of ROS as follows: <http://wiki.ros.org/melodic/Installation/Ubuntu>

### ROS

You must be familiar with ROS, which is the framework used in this project. If you are not, then visit again the wiki of ROS and complete the tutorials from 1 to 12 (you can skip the C++ units since the whole project is programmed in Python): <http://wiki.ros.org/ROS/Tutorials>


## Folders

In this section you will be explained how the project is structured:

* **src**: this folder contains all the software which run the simulator. Here you will find the codes of the circuits and the vehicles.
* **world**: this folder contains the XML description of the visual world developed in Gazebo. If you want to modify the visual look of the world, you can either change some lines of the XML file or modify it directly in Gazebo.
* **rviz**: this is just a folder for Rviz, which another simulator (the one you will use to drive your own car). Dont you mind about this folder.
* **prius_description**: the car you are driving in the simulator (Toyota Prius) is placed in this folder. Here you will find the URDF file of the vehicle. The URDF file (Unified Robot Description Format) is the description of the robot itself. You can remove some parts of the car, such as sensors, by placing some comments on the code.
* **launch**: this folders contains the code to launch the simulator at once. You will be explained how to do this in later sections.
* **models**: the folder which contains the XML descriptions of the roads of the circuit. Pay attention because this is very important as long as you want to run the simulator succesfully. **YOU MUST PLACE THIS FOLDER ON THE .gazebo FOLDER**. This folder is a hidden folder of Ubuntu, you will realise of it by pressing Ctrl+H on your personal directory.
* **PythonRobotics**: appart from being a folder of this repository, this is a repository as well. The algorithm of the autonomous cars is based on the idea of PythonRobotics.

## Instructions to run the simulator

