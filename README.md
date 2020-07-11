# Gazebo-simulator 

Welcome to the Gazebo simulator. Gazebo is an Open Source simulator very useful for control and path planning applicationes, such as autonomous driving among others. This project consists on a simulator of autonomous vehicles using Open Source tools, such as ROS, Linux and Gazebo. The main idea of the project was to develop an Open Source simulation environment where you, as a driver, can handle your own car (the one shown on the image below) in an urban circuit with more vehicles.


![Prius](https://camo.githubusercontent.com/9cadf02ca935944b325ed7de6334899b90863534/68747470733a2f2f7777772e6f7372666f756e646174696f6e2e6f72672f776f72647072657373322f77702d636f6e74656e742f75706c6f6164732f323031372f30362f70726975735f726f756e6461626f75745f657869742e706e67)

## Pre requisites

If you have reached this website, you may want to run this project on your computer and try it out yourself. Well, first of all, you and your computer need to meet some conditions. Some of the following sections are a must if you want to run the simulator on your computer. If you are already done with this part, you can skip it and go ahead with the following.

### Software

* Ubuntu 18.04
* Having installed ROS (melodic version). If you dont have set up ROS on your computer, visit the wiki of ROS as follows: <http://wiki.ros.org/melodic/Installation/Ubuntu>
* 8Gb RAM

Of course, in order to drive your user car you will need a Xbox 360/PlayStation remote control


### ROS

You must be familiar with ROS since is the framework used in this project. If you are not, then visit again the wiki of ROS and complete the tutorials from 1 to 12 (you can skip the C++ units since the whole project is programmed in Python): <http://wiki.ros.org/ROS/Tutorials>


## Folders

In this section it will be explained how the project is structured:

* **src**: this folder contains all the software which run the simulator. Here you will find the codes of the circuits and the vehicles.
* **world**: this folder contains the XML description of the visual world developed in Gazebo. If you want to modify the visual look of the world, you can either change some lines of the XML file or modify it directly in Gazebo.
* **rviz**: this is just a folder for Rviz, which another simulator (the one you will use to drive your own car). Dont you mind about this folder.
* **prius_description**: the car you are driving in the simulator (Toyota Prius) is placed in this folder. Here you will find the URDF file of the vehicle. The URDF file (Unified Robot Description Format) is the description of the robot itself. You can remove some parts of the car, such as sensors, by placing some comments on the code.
* **launch**: this folders contains the code to launch the simulator at once. You will be explained how to do this in later sections.
* **models**: the folder which contains the XML descriptions of the roads of the circuit. Pay attention because this is very important as long as you want to run the simulator succesfully. **YOU MUST PLACE THIS FOLDER ON THE ".gazebo" FOLDER**, which is placed on your personal path (in my case, /home/jesus). This folder is a hidden folder of Ubuntu and you will realise of it by pressing Ctrl+H.
* **PythonRobotics**: appart from being a folder of this repository, this is a repository as well. The algorithm of the autonomous cars is based on the idea of PythonRobotics.

## Instructions to run the simulator

This is the easiest part. In order to run the simulator, you only need to execute the following command on your terminal:

`$ roslaunch Gazebo-simulator Gazebo.launch`

This command will automatically launch:

* Five autonomous vehicles (three cars, one ambulance and one bus)
* The vehicle you drive
* The simulation environment in Gazebo
* The ROS node you need to drive your car

## PythonRobotics

PythonRobotics is the repository of GitHub which the algorithm of the vehicles of Gazebo is based on. You can visit this repository on the following link: <https://github.com/AtsushiSakai/PythonRobotics> 

PythonRobotics uses the Frenet coordinates system proposed in [this paper](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf)

![Frenet](https://raw.githubusercontent.com/AtsushiSakai/PythonRoboticsGifs/master/PathPlanning/FrenetOptimalTrajectory/animation.gif)

