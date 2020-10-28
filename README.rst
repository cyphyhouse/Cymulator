################################################
Cymulator: Gazebo based Simulator for CyPhyHouse
################################################


.. image:: https://img.shields.io/github/license/cyphyhouse/Cymulator
    :target: LICENSE
    :alt: License


Cymulator is a Gazebo_ based simulation environment for CyPhyHouse
project. In Cymulator, the same distributed Koord_ program along with our middleware_ can be
deployed to multiple simulated drones and cars in Gazebo simulator. You can test
and visualize your distributed robot control algorithms without buying drones
and cars!

.. _Gazebo: http://gazebosim.org/
.. _Koord: https://github.com/cyphyhouse/KoordLanguage
.. _middleware: https://github.com/cyphyhouse/CyPyHous3


*************************
Website and Documentation
*************************

Cymulator is part of the CyPhyHouse project and currently not executable as a
standalone application. Please visit following websites for detail usage.

Broad overview of CyPhyHouse project is available at:

  https://cyphyhouse.github.io/

Or you can visit our website for documentation:

  https://cyphyhouse.rtfd.io/



*******
License
*******

Cymulator is licensed under the terms of the NCSA License (see the file
`LICENSE <LICENSE>`_).


.. include-start-after

************
Installation
************

The installation steps below are also assembled in `this shell script <installation/install.sh>`_ that should work for Ubuntu 16.04.
These commands requires `sudo` permission. Please run them with caution.

#. Install ROS Kinetic and create a workspace for catkin. We assume it is under `catkin_ws`.

   - `ROS Kinetic Ubuntu <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_
   - `Creating a workspace for catkin <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_

#. Install Gazebo 9 for ROS Kinetic

   .. code-block:: shell

      sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
      wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
      sudo apt-get update
      sudo apt install -y \
           ros-kinetic-gazebo9-ros ros-kinetic-gazebo9-ros-control \
           ros-kinetic-gazebo9-plugins ros-kinetic-gazebo9-ros-pkgs

#. Install required ROS packages available on APT

   .. code-block:: shell

      sudo apt install -y \
           ros-kinetic-ackermann-msgs ros-kinetic-geographic-msgs ros-kinetic-serial \
           ros-kinetic-ros-control ros-kinetic-ros-controllers \
           ros-kinetic-hector-localization ros-kinetic-hector-models \
           ros-kinetic-geometry2 ros-kinetic-robot

#. Install other system packages available on APT

   .. code-block:: shell

      sudo apt install -y git
      sudo apt install -y cppad coinor-libipopt-dev  # For MPC controller
      sudo apt install -y python3 python3-pip

#. Install required Python packages available on PyPI

   .. code-block:: shell

      pip3 install --user pip --upgrade
      pip3 install --user \
           catkin_pkg rospkg \
           empy numpy scipy \
           defusedxml netifaces \
           pathlib pyyaml

#. Inside the `catkin_ws/src` directory of your catkin workspace clone the following repos:

   .. code-block:: shell

      git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git --branch kinetic-devel
      git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git --branch kinetic-devel
      git clone https://github.com/cyphyhouse/racecar.git --branch RacecarJTransitory
      git clone https://github.com/cyphyhouse/racecar_gazebo.git --branch master
      git clone https://github.com/cyphyhouse/Decawave.git --branch for-cymulator
      git clone https://github.com/cyphyhouse/Cymulator.git --branch master


Compile using catkin_make
=========================

7. Run these commands under your `catkin_ws` directory to compile relevant ROS packages in the cloned repositories.

   .. code-block:: shell

      source /opt/ros/kinetic/setup.bash
      catkin_make --only-pkg-with-deps cym_gazebo --cmake-args -DPYTHON_VERSION=3.5  # Build only cym_gazebo with Python>=3.5


Compile using colcon
====================

7. Run these commands under your `catkin_ws` directory to compile only relevant ROS packages in `catkin_ws/src`.

   .. code-block:: shell

      source /opt/ros/kinetic/setup.bash
      colcon build --base-paths src/* --packages-up-to cym_gazebo --cmake-args -DPYTHON_VERSION=3.5

