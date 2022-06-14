#!/usr/bin/env python

import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gym.utils import seeding
from gym.envs.registration import register
import copy
import math
import os

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState
from rosgraph_msgs.msg import Clock
from openai_ros import robot_gazebo_env
from openai_ros.openai_ros_common import ROSLauncher


class CartPoleEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(
        self, control_type, ros_ws_abspath
    ):

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="cartpole_description",
                    launch_file_name="put_robot_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        self.publishers_array = []
        self._base_pub = rospy.Publisher(
            '/cartpole_v0/foot_joint_velocity_controller/command', Float64, queue_size=1)
        self._pole_pub = rospy.Publisher(
            '/cartpole_v0/pole_joint_velocity_controller/command', Float64, queue_size=1)
        self.publishers_array.append(self._base_pub)
        self.publishers_array.append(self._pole_pub)

        rospy.Subscriber("/cartpole_v0/joint_states",
                         JointState, self.joints_callback)

        self.control_type = control_type
        if self.control_type == "velocity":
            self.controllers_list = ['joint_state_controller',
                                     'pole_joint_velocity_controller',
                                     'foot_joint_velocity_controller',
                                     ]

        elif self.control_type == "position":
            self.controllers_list = ['joint_state_controller',
                                     'pole_joint_position_controller',
                                     'foot_joint_position_controller',
                                     ]

        elif self.control_type == "effort":
            self.controllers_list = ['joint_state_controller',
                                     'pole_joint_effort_controller',
                                     'foot_joint_effort_controller',
                                     ]

        self.robot_name_space = "cartpole_v0"
        self.reset_controls = True

        # Seed the environment
        self._seed()
        self.steps_beyond_done = None

        super(CartPoleEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space=self.robot_name_space,
            reset_controls=self.reset_controls
        )

    def joints_callback(self, data):
        self.joints = data

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # RobotEnv methods
    # ----------------------------

    def _env_setup(self, initial_qpos):
        self.init_internal_vars(self.init_pos)
        self.set_init_pose()
        self.check_all_systems_ready()

    def init_internal_vars(self, init_pos_value):
        self.pos = [init_pos_value]
        self.joints = None

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._base_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug(
                "No susbribers to _base_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_base_pub Publisher Connected")

        while (self._pole_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug(
                "No susbribers to _pole_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_pole_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def _check_all_systems_ready(self, init=True):
        self.base_position = None
        while self.base_position is None and not rospy.is_shutdown():
            try:
                self.base_position = rospy.wait_for_message(
                    "/cartpole_v0/joint_states", JointState, timeout=1.0)
                rospy.logdebug(
                    "Current cartpole_v0/joint_states READY=>"+str(self.base_position))
                if init:
                    # We Check all the sensors are in their initial values
                    positions_ok = all(
                        abs(i) <= 1.0e-02 for i in self.base_position.position)
                    velocity_ok = all(
                        abs(i) <= 1.0e-02 for i in self.base_position.velocity)
                    efforts_ok = all(
                        abs(i) <= 1.0e-01 for i in self.base_position.effort)
                    base_data_ok = positions_ok and velocity_ok and efforts_ok
                    rospy.logdebug(
                        "Checking Init Values Ok=>" + str(base_data_ok))
            except:
                rospy.logerr(
                    "Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
        rospy.logdebug("ALL SYSTEMS READY")

    def move_joints(self, joints_array):
        joint_value = Float64()
        joint_value.data = joints_array[0]
        rospy.logdebug("Single Base JointsPos>>"+str(joint_value))
        self._base_pub.publish(joint_value)

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message(
                    "/clock", Clock, timeout=1.0)
                rospy.logdebug("Current clock_time READY=>" +
                               str(self.clock_time))
            except:
                rospy.logdebug(
                    "Current clock_time not ready yet, retrying for getting Current clock_time")
        return self.clock_time
