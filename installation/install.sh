#!/bin/bash

cd ~

if [ ! "$(rosversion -d)" ]; then
    # Setup your sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    # Set up your keys
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    # Installation
    ## First, make sure your Debian package index is up-to-date:
    sudo apt-get update
    ## Install ROS Kinetic Desktop Full: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception
    sudo apt-get -y install ros-kinetic-desktop-full

    # Initialize rosdep
    sudo rosdep init
    rosdep update
fi


# Install Gazebo
if [ ! -x "$(command -v gazebo)" ]; then
    curl -sSL http://get.gazebosim.org | sh
fi

# Other ROS packages

sudo apt install -y \
    ros-kinetic-ackermann-msgs ros-kinetic-geographic-msgs ros-kinetic-serial \
    ros-kinetic-ros-control ros-kinetic-ros-controllers \
    ros-kinetic-tf2-bullet

# Other System packages
sudo apt install -y cppad coinor-libipopt-dev  # For MPC controller

# Python Packages
sudo apt install -y python3 python3-pip
pip3 install --user pip --upgrade
pip3 install --user \
    catkin_pkg catkin-pkg-modules rospkg-modules \
    empy numpy \
    pathlib pyyaml


# TODO Download the catkin workspace
echo "Please find Chiao to get an updated zip file with catkin workspace for now"
exit 0

echo "------------------------ dependency installation finished -----------------------------"


# Compile all ROS packages
cd ~/catkin_ws3
source /opt/ros/kinetic/setup.bash
catkin_make --cmake-args -DPYTHON_VERSION=3.5 -DCMAKE_CXX_STANDARD=14  # Build with Python>=3.5 and c++14


# Development Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws3/devel/setup.bash" >> ~/.bashrc
echo "export SVGA_VGPU10=0" >> ~/.bashrc  # For running Gazebo in virtual machine
source ~/.bashrc
