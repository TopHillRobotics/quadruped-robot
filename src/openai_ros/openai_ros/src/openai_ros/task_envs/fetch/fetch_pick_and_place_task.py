from gym import utils
import copy
import rospy
import numpy as np
from gym import spaces
from openai_ros.robot_envs import fetchsimple_env
from gym.envs.registration import register
import numpy as np
from sensor_msgs.msg import JointState
from openai_ros.openai_ros_common import ROSLauncher
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
import os


class FetchPickAndPlaceEnv(fetchsimple_env.FetchSimpleEnv, utils.EzPickle):
    def __init__(self):

        # Launch the Task Simulated-Environment
        # This is the path where the simulation files are,
        # the Task and the Robot gits will be downloaded if not there

        ros_ws_abspath = rospy.get_param("/fetch/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="fetch_simple_description",
                    launch_file_name="start_HER_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file relative to this TaskEnvironment
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/fetch/config",
                               yaml_file_name="fetchsimple_test.yaml")

        super(FetchPickAndPlaceEnv, self).__init__(ros_ws_abspath)

        rospy.logdebug("Entered FetchPickAndPlaceEnv Env")
        self.get_params()

        self.action_space = spaces.Discrete(self.n_actions)

        observations_high_range = np.array(
            self.upper_array_observations)
        observations_low_range = np.array(
            self.lower_array_observations)

        self.observation_space = spaces.Box(
            observations_low_range, observations_high_range)

    def get_params(self):
        # get configuration parameters

        self.n_actions = rospy.get_param('/fetch/n_actions')
        self.n_max_iterations = rospy.get_param('/fetch/max_iterations')

        self.init_pos = rospy.get_param('/fetch/init_pos')

        init_pos_dict = rospy.get_param('/fetch/init_pos')
        self.init_pos = [init_pos_dict["bellows_joint"],
                         init_pos_dict["elbow_flex_joint"],
                         init_pos_dict["forearm_roll_joint"],
                         init_pos_dict["head_pan_joint"],
                         init_pos_dict["head_tilt_joint"],
                         init_pos_dict["l_gripper_finger_joint"],
                         init_pos_dict["r_gripper_finger_joint"],
                         init_pos_dict["shoulder_lift_joint"],
                         init_pos_dict["shoulder_pan_joint"],

                         init_pos_dict["torso_lift_joint"],
                         init_pos_dict["upperarm_roll_joint"],
                         init_pos_dict["wrist_flex_joint"],
                         init_pos_dict["wrist_roll_joint"]
                         ]

        goal_pos_dict = rospy.get_param('/fetch/goal_pos')
        self.goal_pos = [goal_pos_dict["elbow_flex_joint"],
                         goal_pos_dict["shoulder_lift_joint"],
                         goal_pos_dict["shoulder_pan_joint"]]

        self.position_delta = rospy.get_param('/fetch/position_delta')
        self.reached_goal_reward = rospy.get_param(
            '/fetch/reached_goal_reward')

        upper_array, lower_array = self.get_joint_limits()
        self.upper_array_observations = [
            upper_array[1], upper_array[7], upper_array[8]]
        self.lower_array_observations = [
            lower_array[1], lower_array[7], lower_array[8]]

        self.n_observations = len(self.upper_array_observations)

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        The Simulation will be unpaused for this purpose.
        """
        # Check because it seems its not being used
        rospy.logdebug("Init Pos:")
        rospy.logdebug(self.init_pos)

        """
        # Init Joint Pose
        rospy.logdebug("Moving To SETUP Joints ")
        self.movement_result = self.set_trajectory_joints(self.init_pos)
        """

        # We test the Desired Goal

        # INIT POSE
        rospy.logdebug("Moving To Init Pose ")
        self.move_to_init_pose()
        self.last_action = "INIT"

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        The simulation will be paused, therefore all the data retrieved has to be
        from a system that doesnt need the simulation running, like variables where the
        callbackas have stored last know sesnor data.
        :return:
        """
        rospy.logdebug("Init Env Variables...")
        self.interations_done = 0
        rospy.logdebug("Init Env Variables...END")

    def _set_action(self, action):

        delta_gripper_target = [0.0]*len(self.init_pos)
        rospy.logwarn("==== ROBOT ENV RECEIVED ACTION: "+str(action))

        delta_gripper_target[1] += action[0]
        delta_gripper_target[7] += action[1]
        delta_gripper_target[8] += action[2]

        # # We convert action ints in increments/decrements of one of the axis XYZ
        # if action == 0:  # elbow_flex_joint+
        #     delta_gripper_target[1] += self.position_delta
        #     self.last_action = "elbow_flex_joint+"
        # elif action == 1:  # elbow_flex_joint-
        #     delta_gripper_target[1] -= self.position_delta
        #     self.last_action = "elbow_flex_joint-"
        # elif action == 2:  # shoulder_lift_joint+
        #     delta_gripper_target[7] += self.position_delta
        #     self.last_action = "shoulder_lift_joint+"
        # elif action == 3:  # shoulder_lift_joint-
        #     delta_gripper_target[7] -= self.position_delta
        #     self.last_action = "shoulder_lift_joint-"
        # elif action == 4:  # shoulder_pan_joint+
        #     delta_gripper_target[8] += self.position_delta
        #     self.last_action = "shoulder_pan_joint+"
        # elif action == 5:  # shoulder_pan_joint-
        #     delta_gripper_target[8] -= self.position_delta
        #     self.last_action = "shoulder_pan_joint-"

        self.movement_result = self.set_trajectory_joints(delta_gripper_target)

        # rospy.logwarn("END Set Action ==>" + str(action) +
        #               ", NAME=" + str(self.last_action))

    def _get_obs(self):
        """
        It returns the Position of the TCP/EndEffector as observation.
        And the distance from the desired point
        Orientation for the moment is not considered
        """

        joints_position = self.get_joints_position()
        obs_joints_position = {
            'observation': np.array([joints_position[1]]),
            'achieved_goal': np.array([joints_position[7]]),
            'desired_goal': np.array([joints_position[8]]),
        }

        # obs_joints_position = [joints_position[1],
        #                        joints_position[7], joints_position[8]]

        rospy.logdebug("OBSERVATIONS====>>>>>>>"+str(obs_joints_position))

        return obs_joints_position

    def _is_done(self, observations):
        """
        If the latest Action didnt succeed, it means that tha position asked was imposible therefore the episode must end.
        It will also end if it reaches its goal.
        """

        done = np.allclose(a=observations,
                           b=self.goal_pos,
                           atol=0.2)

        self.interations_done += 1

        if self.interations_done >= self.n_max_iterations:
            done = True

        return done

    def _compute_reward(self, observations, done):
        """
        We punish each step that it passes without achieveing the goal.
        Punishes differently if it reached a position that is imposible to move to.
        Rewards getting to a position close to the goal.
        """
        reward = 1.0 / \
            (np.linalg.norm(np.array(observations) - np.array(self.goal_pos)))
        if done:
            reward += self.reached_goal_reward
        rospy.logwarn(">>>REWARD>>>"+str(reward))

        return reward
