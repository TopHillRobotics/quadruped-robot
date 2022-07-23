from gym import utils
import copy
import rospy
from gym import spaces
from openai_ros.robot_envs import fetch_env
from gym.envs.registration import register
import numpy as np
from sensor_msgs.msg import JointState
from openai_ros.openai_ros_common import ROSLauncher
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
import os


class FetchPushEnv(fetch_env.FetchEnv, utils.EzPickle):
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
                               yaml_file_name="fetch_push.yaml")

        self.get_params()

        # TODO: this must be continuous action space... don't follow the old implementation.
        self.action_space = spaces.Discrete(self.n_actions)

        observations_high_range = np.array(
            [self.position_ee_max]*self.n_observations)
        observations_low_range = np.array(
            [self.position_ee_min]*self.n_observations)

        observations_high_dist = np.array([self.max_distance])
        observations_low_dist = np.array([0.0])

        high = np.concatenate(
            [observations_high_range, observations_high_dist])
        low = np.concatenate([observations_low_range, observations_low_dist])

        self.observation_space = spaces.Box(low, high)

        # TODO: Clean up
        # fetch_env.FetchEnv.__init__(self)
        super(FetchPushEnv, self).__init__(ros_ws_abspath)

    def get_params(self):
        """
        get configuration parameters

        """
        self.sim_time = rospy.get_time()
        self.n_actions = rospy.get_param('/fetch/n_actions')
        self.n_observations = rospy.get_param('/fetch/n_observations')
        self.position_ee_max = rospy.get_param('/fetch/position_ee_max')
        self.position_ee_min = rospy.get_param('/fetch/position_ee_min')

        self.init_pos = rospy.get_param('/fetch/init_pos')
        self.setup_ee_pos = rospy.get_param('/fetch/setup_ee_pos')
        self.goal_ee_pos = rospy.get_param('/fetch/goal_ee_pos')

        self.position_delta = rospy.get_param('/fetch/position_delta')
        self.step_punishment = rospy.get_param('/fetch/step_punishment')
        self.closer_reward = rospy.get_param('/fetch/closer_reward')
        self.impossible_movement_punishement = rospy.get_param(
            '/fetch/impossible_movement_punishement')
        self.reached_goal_reward = rospy.get_param(
            '/fetch/reached_goal_reward')

        self.max_distance = rospy.get_param('/fetch/max_distance')

        self.goal = np.array(
            [self.goal_ee_pos["x"], self.goal_ee_pos["y"], self.goal_ee_pos["z"]])
        self.rot_ctrl = np.array([1., 0., 1., 0.])

        self.prev_grip_pos = np.zeros(3)
        self.prev_object_pos = np.zeros(3)
        self.prev_object_rot = np.zeros(3)

    def _set_init_pose(self):
        """
        Sets the Robot in its init pose
        The Simulation will be unpaused for this purpose.
        """
        if not self.set_trajectory_joints(self.init_pos):
            assert False, "Initialisation is failed...."

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
        rospy.logdebug("Init Env Variables...END")

    def _set_action(self, action):
        rospy.logwarn("=== Action: {}".format(action))
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        """
        Since the implementation of self.set_trajectory_ee(in fetch_env.py) ONLY takes the position of the EE(end-effector)
        Action only contains: the destination of the EE in the world frame

        TODO: Talk to Miguel regarding the modification of the basic implementation of self.set_trajectory_ee(in fetch_env.py)

        below code is similar to the original implementaion of OpenAI
        """
        # gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        # action = np.concatenate([pos_ctrl, self.rot_ctrl, gripper_ctrl])

        # TODO: After speak to Miguel, let's not use this, USE above action!!
        action = pos_ctrl

        self.movement_result = self.set_trajectory_ee(action)
        if not self.movement_result:
            assert False, "movement_result failed with the action of : " + \
                str(action)

    def _get_obs(self):
        """
        It returns the Position of the TCP/EndEffector as observation.
        And the distance from the desired point
        Orientation for the moment is not considered

        Note:
            - In original code(https://github.com/openai/gym/blob/master/gym/envs/robotics/fetch_env.py#L91),
              the term (xvelp) is used and it means positional velocity in world frame
        """
        grip_pos_v = self.get_ee_pose()
        grip_pos = np.array([grip_pos_v.pose.position.x,
                             grip_pos_v.pose.position.y, grip_pos_v.pose.position.z])
        dt = self.get_elapsed_time()
        # Velocity(position) = Distance/Time
        grip_velp = (grip_pos - self.prev_grip_pos)/dt
        # the pose and rotation of the cube/box on a table
        object_data = self.obj_positions.get_states()
        object_pos = object_data[:3]                         # pose of cube
        object_rot = object_data[3:]                         # rotation of cube
        object_velp = (object_pos - self.prev_object_pos) / \
            dt  # Velocity(position) = Distance/Time
        object_velr = (object_rot - self.prev_object_rot) / \
            dt  # Velocity(rotation) = Rotation/Time
        object_rel_pos = object_pos - grip_pos
        # Unknown meaning of this operation(https://github.com/openai/gym/blob/master/gym/envs/robotics/fetch_env.py#L102)
        object_velp -= grip_velp

        """
        TODO: Ask Miguel the meaning of the two variables below

        1. gripper_state => https: // github.com / openai / gym / blob / master / gym / envs / robotics / fetch_env.py  # L105
        2. gripper_vel   => https: // github.com / openai / gym / blob / master / gym / envs / robotics / fetch_env.py  # L106

        """
        gripper_state = np.zeros(0)  # temp workaround
        gripper_vel = np.zeros(0)    # temp workaround

        achieved_goal = np.squeeze(object_pos.copy())
        obs = np.concatenate([
            grip_pos,                # absolute position of gripper
            object_pos.ravel(),      # absolute position of object
            object_rel_pos.ravel(),  # relative position of object from gripper
            gripper_state,           # gripper state
            object_rot.ravel(),      # rotations of object
            object_velp.ravel(),     # positional velocities of object
            object_velr.ravel(),     # rotational velocities of object
            grip_velp,               # distance between fingers
            gripper_vel,             # velocities of gripper
        ])

        rospy.logdebug("OBSERVATIONS====>>>>>>>"+str(obs))
        # Update the previous properties
        self.prev_grip_pos = grip_pos
        self.prev_object_pos = object_pos
        self.prev_object_rot = object_rot

        return {
            'observation':   obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal':  self.goal.copy(),
        }

    def get_elapsed_time(self):
        """
        Returns the elapsed time since the beginning of the simulation
        Then maintains the current time as "previous time" to calculate the elapsed time again
        """
        current_time = rospy.get_time()
        dt = self.sim_time - current_time
        self.sim_time = current_time
        return dt

    def _is_done(self, observations):
        """
        If the latest Action didnt succeed, it means that tha position asked was imposible therefore the episode must end.
        It will also end if it reaches its goal.
        """

        current_pos = observations[:3]

        done = self.calculate_if_done(
            self.movement_result, self.goal, current_pos)

        return done

    def _compute_reward(self, observations, done):
        """
        Given a success of the execution of the action
        Calculate the reward: binary => 1 for success, 0 for failure
        """
        current_pos = observations[:3]
        # TODO: ask Miguel, why we need this
        new_dist_from_des_pos_ee = observations[-1]

        if self.movement_result:
            position_similar = np.all(np.isclose(
                self.goal, current_pos, atol=1e-02))
            if position_similar:
                reward = self.reached_goal_reward
                rospy.logwarn("Reached a Desired Position!")
            else:
                reward = 0
        else:
            # TODO: Ask Miguel about the purpose of having "self.impossible_movement_punishement"
            reward = self.impossible_movement_punishement
            rospy.logwarn("Reached a TCP position not reachable")
        rospy.logwarn(">>>REWARD>>>"+str(reward))

        return reward

    def calculate_if_done(self, movement_result, goal, current_pos):
        """
        It calculated whather it has finished or not
        """
        done = False

        if movement_result:
            # check if the end-effector located within a threshold
            # TODO: check this threshold is alined with the paper one
            position_similar = np.all(
                np.isclose(goal, current_pos, atol=1e-02))

            if position_similar:
                done = True
                rospy.logdebug("Reached a Desired Position!")
        else:
            # or if the end-effector reaches the maximum strechable point
            done = True
            rospy.logdebug("Reached a TCP position not reachable")

        return done

    def calculate_distance_between(self, v1, v2):
        """
        Calculated the Euclidian distance between two vectors given as python lists.
        """
        dist = np.linalg.norm(np.array(v1) - np.array(v2))
        return dist
