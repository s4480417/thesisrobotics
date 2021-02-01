# Thesis Robotics Repository
Thesis project by Miguel Marco Valencia

Student Number: 44804172

Project Commencement: Nov 2020

# Project Title:
Medical robotics as a tool for measuring movement quality in neurorehabilitation.
# Project Description:
The project aims to assess the validity of clinical tests conducted by a pair of 6-DOF serial robots.

The clinical tests will be used to objectively and consistently measure the performance of a patients who have recently suffered from a stroke.

The robotic arms which are used for the first iteration are the Kinova Jaco2 (also called Gen2) arms, which are attached to the Kinova Movo robot.

The framework will be designed to be robust and be used with other arms (e.g. the Franka Emika Pandas).
# Table of Contents
1. How To

1.1. Prerequisites

1.2. Installation
# 1 How To
## 1.1 Prerequisites
- Install Ubuntu 18.04 (Bionic Beaver) (Required)
- Install [ROS Melodic](http://wiki.ros.org/melodic) (Required)
- Download the [Kinova ROS Stack](https://github.com/Kinovarobotics/kinova-ros) (for using Kinova Gen2/Jaco2 Robots)
- Build and install [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) from source
### 1.1.1 ROS Installation
- After a clean installation of Ubuntu 18.04
- sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.li
- sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
- sudo apt update
- sudo apt install ros-melodic-desktop-full
- echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
- source ~/.bashrc
- sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
- sudo apt install python-rosdep
- sudo rosdep init
- rosdep update
- sudo apt-get install ros-melodic-moveit
## 1.2 Installation
- Create and enter a catkin workspace (e.g. ~/catkin_ws)
```console
mkdir ~/catkin_ws && cd ~/catkin_ws
```
- Create a source directory within your workspace
```console
mkdir ./src && cd ./src
```
- Clone this git repository
```console
git clone https://github.com/s4480417/thesisrobotics.git
```
- Enter the root of your workspace
```console
cd ~/catkin_ws
```
- Run catkin_make
```console
catkin_make
```
## Using the Stack
