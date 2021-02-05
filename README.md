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
```console
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.li
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
sudo apt-get install ros-melodic-moveit
```
### 1.1.2 CUDA and cuDNN Installation
- Note: This was installed on a computer with the GeForce RTX 2080
- Installing CUDA
```console
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget http://developer.download.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda-repo-ubuntu1804-10-1-local-10.1.243-418.87.00_1.0-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu1804-10-1-local-10.1.243-418.87.00_1.0-1_amd64.deb
sudo apt-key add /var/cuda-repo-10-1-local-10.1.243-418.87.00/7fa2af80.pub
sudo apt-get update
sudo apt-get -y install cuda
```
- Installing cuDNN
Go to the NVIDIA developer website [here](https://developer.nvidia.com/rdp/cudnn-download).

Click on: **Download cuDNN v7.6.5 (November 5th, 2019), for CUDA 10.1**

Download:
  - cuDNN Runtime Library for Ubuntu18.04 (Deb)
  - cuDNN Developer Library for Ubuntu18.04 (Deb)
  - cuDNN Code Samples and User Guide for Ubuntu18.04 (Deb)
 
In a terminal, go to the folder you downloaded these to.
 
Run these commands:
**Runtime Library**
```console
    sudo dpkg -i libcudnn7_7.6.5.32-1+cuda10.1_amd64.deb
```
**Developer Library**
```console
    sudo dpkg -i libcudnn7-dev_7.6.5.32-1+cuda10.1_amd64.deb
```
**Code Samples**
```console
    sudo dpkg -i libcudnn7-doc_7.6.5.32-1+cuda10.1_amd64.deb
```
Verification:
Go to the MNIST example code:
```console
    cd /usr/src/cudnn_samples_v7/mnistCUDNN/
```
Compile the MNIST example:
```console
    sudo make clean && sudo make
```
Run the MNIST example:
```console
    ./mnistCUDNN
```
You should see:
```console
  Test passed!
```

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
