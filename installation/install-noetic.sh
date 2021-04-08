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
    ## Install ROS Noetic Desktop Full: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception
    sudo apt-get -y install ros-noetic-desktop-full

    # Initialize rosdep
    sudo rosdep init
    rosdep update
fi

# Other ROS packages

sudo apt install -y \
    ros-noetic-ackermann-msgs ros-noetic-geographic-msgs \
    ros-noetic-eigen-stl-containers \
    ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-hector-models \
    ros-noetic-geometry2 ros-noetic-robot

# Other System packages
sudo apt install -y git
sudo apt install -y cppad coinor-libipopt-dev  # For MPC controller

# Python Packages
sudo apt install -y python3 python3-pip
pip3 install --user pip --upgrade
pip3 install --user \
    catkin_pkg rospkg \
    empy numpy scipy\
    defusedxml netifaces \
    pathlib pyyaml

echo "-------------------- System dependency installation finished -------------------------"


# Download source code of all needed ROS packages
mkdir -p catkin_ws3/src
cd catkin_ws3/src
# TODO look into using .rosinstall to automatically download these repositories
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git --branch kinetic-devel
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git --branch kinetic-devel
git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization.git
git clone https://github.com/cyphyhouse/racecar.git --branch RacecarJTransitory
git clone https://github.com/cyphyhouse/racecar_gazebo.git --branch master
git clone https://github.com/cyphyhouse/Decawave.git --branch for-cymulator
git clone https://github.com/cyphyhouse/Cymulator.git --branch master

# Compile all ROS packages
cd ..  # Go back to catkin workspace
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps cym_gazebo --cmake-args -DPYTHON_VERSION=3.5  # Build only cym_gazebo with Python>=3.5


# (Optional) Development Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws3/devel/setup.bash" >> ~/.bashrc
echo "export SVGA_VGPU10=0" >> ~/.bashrc  # For running Gazebo in virtual machine
source ~/.bashrc
