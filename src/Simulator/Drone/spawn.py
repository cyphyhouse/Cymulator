#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse, DeleteModel
from copy import deepcopy
from tf.transformations import quaternion_from_euler


class Spawn:

    def __init__(self):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.count = 0
        self.paths = {}

        self.sdf_cube = """<?xml version="1.0" ?>
        <sdf version="1.4">
          <model name="MODELNAME">
            <static>0</static>
            <link name="link">
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.01</ixx>
                  <ixy>0.0</ixy>
                  <ixz>0.0</ixz>
                  <iyy>0.01</iyy>
                  <iyz>0.0</iyz>
                  <izz>0.01</izz>
                </inertia>
              </inertial>
              <collision name="stairs_collision0">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                  <box>
                    <size>SIZEXYZ</size>
                  </box>
                </geometry>
                <surface>
                  <bounce />
                  <friction>
                    <ode>
                      <mu>1.0</mu>
                      <mu2>1.0</mu2>
                    </ode>
                  </friction>
                  <contact>
                    <ode>
                      <kp>10000000.0</kp>
                      <kd>1.0</kd>
                      <min_depth>0.0</min_depth>
                      <max_vel>0.0</max_vel>
                    </ode>
                  </contact>
                </surface>
              </collision>
              <visual name="stairs_visual0">
                <pose>0 0 0 0 0 0</pose>
                <geometry>
                  <box>
                    <size>SIZEXYZ</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Wood</name>
                  </script>
                </material>
              </visual>
              <velocity_decay>
                <linear>0.000000</linear>
                <angular>0.000000</angular>
              </velocity_decay>
              <self_collide>0</self_collide>
              <kinematic>0</kinematic>
              <gravity>1</gravity>
            </link>
          </model>
        </sdf>
        """

    def create_point(self, x, y, z, idx):
        req = self.create_cube_request("waypoint"+str(idx),
                                   x, y, z,  # position
                                   0.0, 0.0, 0.0,  # rotation
                                   0.2, 0.2, 0.01)  # size
        self.spawn_model(req)
        self.count += 1
        rospy.sleep(0.01)
        return idx

    def delete_point(self, idx):
        self.delete_model("waypoint"+str(idx))
        rospy.sleep(0.01)

    def create_cube_request(self, modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
        """Create a SpawnModelRequest with the parameters of the cube given.
        modelname: name of the model for gazebo
        px py pz: position of the cube (and it's collision cube)
        rr rp ry: rotation (roll, pitch, yaw) of the model
        sx sy sz: size of the cube"""
        cube = deepcopy(self.sdf_cube)
        # Replace size of model
        size_str = str(round(sx, 3)) + " " + \
            str(round(sy, 3)) + " " + str(round(sz, 3))
        cube = cube.replace('SIZEXYZ', size_str)
        # Replace modelname
        cube = cube.replace('MODELNAME', str(modelname))

        req = SpawnModelRequest()
        req.model_name = modelname
        req.model_xml = cube
        req.initial_pose.position.x = px
        req.initial_pose.position.y = py
        req.initial_pose.position.z = pz

        q = quaternion_from_euler(rr, rp, ry)
        req.initial_pose.orientation.x = q[0]
        req.initial_pose.orientation.y = q[1]
        req.initial_pose.orientation.z = q[2]
        req.initial_pose.orientation.w = q[3]

        return req


if __name__ == '__main__':
    rospy.init_node("Spawn_model", anonymous=True)
    track = Spawn()
    for i in range(1,10):
        track.create_point(i, i, 0, i)

    for i in range(1, 10):
        track.delete_point(i)