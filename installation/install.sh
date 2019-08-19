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
    # We don't install ros-kinetic-desktop-full because we want to use Gazebo 9
    sudo apt-get -y install ros-kinetic-desktop

    # Initialize rosdep
    sudo rosdep init
    rosdep update
fi


# Install Gazebo 9
if [ ! -x "$(command -v gazebo)" ]; then
    # curl -sSL http://get.gazebosim.org | sh
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt install -y \
        ros-kinetic-gazebo9-ros ros-kinetic-gazebo9-ros-control \
        ros-kinetic-gazebo9-plugins ros-kinetic-gazebo9-ros-pkgs
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
    catkin_pkg rospkg \
    empy numpy \
    pathlib pyyaml


# TODO Download the catkin workspace
echo "Please find Chiao to get an updated zip file with catkin workspace for now"
exit 0

echo "------------------------ dependency installation finished -----------------------------"


# Compile all ROS packages
cd ~/catkin_ws3
source /opt/ros/kinetic/setup.bash
catkin_make --cmake-args -DPYTHON_VERSION=3.5  # Build with Python>=3.5


# Development Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws3/devel/setup.bash" >> ~/.bashrc
echo "export SVGA_VGPU10=0" >> ~/.bashrc  # For running Gazebo in virtual machine
source ~/.bashrc
