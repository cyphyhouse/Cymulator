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

The installation steps below are assembled in ``installation/install-noetic.sh`` which we tested on Ubuntu 20.04.
We also provide scripts for few different combinations of ROS, Gazebo, and Ubuntu versions under ``installation/`` folder.
These commands requires ``sudo`` permission. Please run them with caution.

#. Install ROS Noetic with Gazebo 11 and create a workspace for catkin. We assume it is under ``cym_ws``.

   - `Ubuntu Install of ROS Noetic  <http://wiki.ros.org/noetic/Installation/Ubuntu>`_
   
     * Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
   
   - `Creating a workspace for catkin <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_

#. Install required ROS packages available on APT

   .. code-block:: shell

      sudo apt install -y \
          ros-noetic-ackermann-msgs ros-noetic-geographic-msgs \
          ros-noetic-eigen-stl-containers \
          ros-noetic-ros-control ros-noetic-ros-controllers \
          ros-noetic-hector-models \
          ros-noetic-geometry2 ros-noetic-robot

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
           pathlib pyyaml \
           vcstool


Clone required Git repositories
===============================

6. Inside the ``cym_ws/src`` directory, clone Cymulator repository.
   Then, clone dependent git repositories specified in ``vcstool.repos.yml`` using `vcstool <https://pypi.org/project/vcstool/>`_:

   .. code-block:: shell

      git clone https://github.com/cyphyhouse/Cymulator.git --branch master
      vcs import --input Cymulator/vcstool.repos.yml

Alternatively, you can manually clone the repositories at the versions specified in ``vcstool.repos.yml`` without using ``vcstool``.


Compile using catkin_make
=========================

7. Run these commands under your ``cym_ws`` directory to compile relevant ROS packages in the cloned repositories.

   .. code-block:: shell

      source /opt/ros/kinetic/setup.bash
      catkin_make --only-pkg-with-deps cym_gazebo  # Build only packages required by cym_gazebo


(Alternative) Compile using colcon
==================================

`Colcon <https://colcon.readthedocs.io>`_ is an alternative build system used by `Amazon AWS RoboMaker <https://aws.amazon.com/robomaker/>`_.
Here are the commands to build with ``colcon``.

7. Run these commands under your ``cym_ws`` directory to compile only relevant ROS packages in ``cym_ws/src``.

   .. code-block:: shell

      source /opt/ros/kinetic/setup.bash
      colcon build --base-paths src/* --packages-up-to cym_gazebo


(Optional) Additional Gazebo Worlds
===================================

You can download other Gazebo worlds. Cymulator relies on ROS Gazebo package to load Gazebo simulator, and it should be able to load your custom world.
Some interesting world files from other opensource projects are provided below

* https://docs.px4.io/master/en/simulation/gazebo_worlds.html
