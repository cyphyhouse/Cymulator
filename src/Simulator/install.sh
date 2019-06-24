#!/bin/bash

cd ~
sudo apt install python3-pip
# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installation
## First, make sure your Debian package index is up-to-date: 
sudo apt-get update
## Install ROS Kinetic Desktop Full: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception 
sudo apt-get install ros-kinetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install Gazebo
curl -sSL http://get.gazebosim.org | sh
echo "export SVGA_VGPU10=0" >> ~/.bashrc
source ~/.bashrc

# Download the catkin workspace
cd ~
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1gMsuEJ0Dsz1ptXX_5-Vps8Rh5Ds-UT0B' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1gMsuEJ0Dsz1ptXX_5-Vps8Rh5Ds-UT0B" -O catkin_ws3.zip && rm -rf /tmp/cookies.txt
unzip catkin_ws3.zip

# Prepare dependencies for catkin_make
# sudo chown $USER -R catkin_ws3
cd ~/catkin_ws3
find ./ -type f -exec sed -i -e "s/mjiang24/$USER/g" {} \;
pip3 install catkin_pkg
pip3 install numpy
pip3 install empy
sudo apt-get install ros-kinetic-geographic-msgs
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-ackermann-msgs
sudo apt-get install python3-yaml
sudo apt-get install python3-catkin-pkg-modules
sudo apt-get install python3-rospkg-modules

# Environment setup
echo "source ~/catkin_ws3/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo "set(CMAKE_CXX_FLAGS '${CMAKE_CXX_FLAGS} -std=c++14')" >>  ~/catkin_ws3/src/CMakeLists.txt

# Build
cd ~/catkin_ws3
catkin_make

