import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import hopper_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class HopperStayUpEnv(hopper_env.HopperEnv):
    def __init__(self):
        """
        Make Hopper Learn how to Stay Up indefenitly
        """

        # Only variable needed to be set here
        """
        For this version, we consider 6 actions
        1-2) Increment/Decrement haa_joint
        3-4) Increment/Decrement hfe_joint
        5-6) Increment/Decrement kfe_joint
        """
        rospy.logdebug("Start HopperStayUpEnv INIT...")

        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/monoped/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="legged_robots_sims",
                    launch_file_name="start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/hopper/config",
                               yaml_file_name="hopper_stay_up.yaml")

        number_actions = rospy.get_param('/monoped/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # Actions and Observations

        self.init_joint_states = Vector3()
        self.init_joint_states.x = rospy.get_param(
            '/monoped/init_joint_states/haa_joint')
        self.init_joint_states.y = rospy.get_param(
            '/monoped/init_joint_states/hfe_joint')
        self.init_joint_states.z = rospy.get_param(
            '/monoped/init_joint_states/kfe_joint')

        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/monoped/desired_point/x")
        self.desired_point.y = rospy.get_param("/monoped/desired_point/y")
        self.desired_point.z = rospy.get_param("/monoped/desired_point/z")
        self.accepted_error_in_des_pos = rospy.get_param(
            "/monoped/accepted_error_in_des_pos")

        self.desired_yaw = rospy.get_param("/monoped/desired_yaw")

        self.joint_increment_value = rospy.get_param(
            "/monoped/joint_increment_value")
        self.init_move_time = rospy.get_param("/monoped/init_move_time", 1.0)
        self.move_time = rospy.get_param("/monoped/move_time", 0.05)
        self.check_position = rospy.get_param("/monoped/check_position", True)

        self.accepted_joint_error = rospy.get_param(
            "/monoped/accepted_joint_error")
        self.update_rate = rospy.get_param("/monoped/update_rate")

        self.dec_obs = rospy.get_param(
            "/monoped/number_decimals_precision_obs")

        self.desired_force = rospy.get_param("/monoped/desired_force")

        self.max_x_pos = rospy.get_param("/monoped/max_x_pos")
        self.max_y_pos = rospy.get_param("/monoped/max_y_pos")

        self.min_height = rospy.get_param("/monoped/min_height")
        self.max_height = rospy.get_param("/monoped/max_height")

        self.distance_from_desired_point_max = rospy.get_param(
            "/monoped/distance_from_desired_point_max")

        self.max_incl_roll = rospy.get_param("/monoped/max_incl")
        self.max_incl_pitch = rospy.get_param("/monoped/max_incl")
        self.max_contact_force = rospy.get_param("/monoped/max_contact_force")

        self.maximum_haa_joint = rospy.get_param("/monoped/maximum_haa_joint")
        self.maximum_hfe_joint = rospy.get_param("/monoped/maximum_hfe_joint")
        self.maximum_kfe_joint = rospy.get_param("/monoped/maximum_kfe_joint")
        self.min_kfe_joint = rospy.get_param("/monoped/min_kfe_joint")

        # We place the Maximum and minimum values of observations
        self.joint_ranges_array = {"maximum_haa": self.maximum_haa_joint,
                                   "minimum_haa_joint": -self.maximum_haa_joint,
                                   "maximum_hfe_joint": self.maximum_hfe_joint,
                                   "minimum_hfe_joint": self.maximum_hfe_joint,
                                   "maximum_kfe_joint": self.maximum_kfe_joint,
                                   "min_kfe_joint": self.min_kfe_joint
                                   }

        high = numpy.array([self.distance_from_desired_point_max,
                            self.max_incl_roll,
                            self.max_incl_pitch,
                            3.14,
                            self.max_contact_force,
                            self.maximum_haa_joint,
                            self.maximum_hfe_joint,
                            self.maximum_kfe_joint,
                            self.max_x_pos,
                            self.max_y_pos,
                            self.max_height
                            ])

        low = numpy.array([0.0,
                           -1*self.max_incl_roll,
                           -1*self.max_incl_pitch,
                           -1*3.14,
                           0.0,
                           -1*self.maximum_haa_joint,
                           -1*self.maximum_hfe_joint,
                           self.min_kfe_joint,
                           -1*self.max_x_pos,
                           -1*self.max_y_pos,
                           self.min_height
                           ])

        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))

        # Rewards
        self.weight_joint_position = rospy.get_param(
            "/monoped/rewards_weight/weight_joint_position")
        self.weight_contact_force = rospy.get_param(
            "/monoped/rewards_weight/weight_contact_force")
        self.weight_orientation = rospy.get_param(
            "/monoped/rewards_weight/weight_orientation")
        self.weight_distance_from_des_point = rospy.get_param(
            "/monoped/rewards_weight/weight_distance_from_des_point")

        self.alive_reward = rospy.get_param("/monoped/alive_reward")
        self.done_reward = rospy.get_param("/monoped/done_reward")

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(HopperStayUpEnv, self).__init__(ros_ws_abspath)

        rospy.logdebug("END HopperStayUpEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the Robot in its init linear and angular speeds
        and lands the robot. Its preparing it to be reseted in the world.
        """

        joints_array = [self.init_joint_states.x,
                        self.init_joint_states.y,
                        self.init_joint_states.z]

        self.move_joints(joints_array,
                         epsilon=self.accepted_joint_error,
                         update_rate=self.update_rate,
                         time_sleep=self.init_move_time,
                         check_position=self.check_position)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        # For Info Purposes
        self.cumulated_reward = 0.0
        # We get the initial pose to mesure the distance from the desired point.
        odom = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(
            odom.pose.pose.position)

    def _set_action(self, action):
        """
        It sets the joints of monoped based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))

        # We get current Joints values
        joint_states = self.get_joint_states()
        joint_states_position = joint_states.position
        rospy.logdebug("get_action_to_position>>>"+str(joint_states_position))

        action_position = [0.0, 0.0, 0.0]

        rospy.logdebug(
            "OLD-JOINT-STATE [haa,hfa,kfe]>>>"+str(joint_states_position))

        if action == 0:  # Increment haa_joint
            rospy.logdebug("Increment haa_joint")
            action_position[0] = joint_states_position[0] + \
                self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
        elif action == 1:  # Decrement haa_joint
            rospy.logdebug("Decrement haa_joint")
            action_position[0] = joint_states_position[0] - \
                self.joint_increment_value
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2]
        elif action == 2:  # Increment hfe_joint
            rospy.logdebug("Increment hfe_joint")
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] + \
                self.joint_increment_value
            action_position[2] = joint_states_position[2]
        elif action == 3:  # Decrement hfe_joint
            rospy.logdebug("Decrement hfe_joint")
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1] - \
                self.joint_increment_value
            action_position[2] = joint_states_position[2]
        elif action == 4:  # Increment kfe_joint
            rospy.logdebug("Increment kfe_joint")
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] + \
                self.joint_increment_value
        elif action == 5:  # Decrement kfe_joint
            rospy.logdebug("Decrement kfe_joint")
            action_position[0] = joint_states_position[0]
            action_position[1] = joint_states_position[1]
            action_position[2] = joint_states_position[2] - \
                self.joint_increment_value

        rospy.logdebug("NEW-JOINT-STATE [haa,hfa,kfe]>>>"+str(action_position))
        rospy.logdebug("JOINT-RANGES>>>"+str(self.joint_ranges_array))

        rospy.logdebug("START ACTION EXECUTE>>>"+str(action))
        # We tell monoped where to place its joints next
        self.move_joints(action_position,
                         epsilon=self.accepted_joint_error,
                         update_rate=self.update_rate,
                         time_sleep=self.move_time,
                         check_position=self.check_position)
        rospy.logdebug("END ACTION EXECUTE>>>"+str(action))

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        HopperEnv API DOCS
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired point in meters
        2) The pitch orientation in radians
        3) the Roll orientation in radians
        4) the Yaw orientation in radians
        5) Force in contact sensor in Newtons
        6-7-8) State of the 3 joints in radians
        9) Height of the Base

        observation = [distance_from_desired_point,
                 base_roll,
                 base_pitch,
                 base_yaw,
                 force_magnitude,
                 joint_states_haa,
                 joint_states_hfe,
                 joint_states_kfe,
                 height_base]
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")

        distance_from_desired_point = self.get_distance_from_desired_point(
            self.desired_point)

        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        force_magnitude = self.get_contact_force_magnitude()

        joint_states = self.get_joint_states()
        joint_states_haa = joint_states.position[0]
        joint_states_hfe = joint_states.position[1]
        joint_states_kfe = joint_states.position[2]

        odom = self.get_odom()
        base_position = odom.pose.pose.position

        observation = []
        observation.append(round(distance_from_desired_point, self.dec_obs))
        observation.append(round(base_roll, self.dec_obs))
        observation.append(round(base_pitch, self.dec_obs))
        observation.append(round(base_yaw, self.dec_obs))
        observation.append(round(force_magnitude, self.dec_obs))
        observation.append(round(joint_states_haa, self.dec_obs))
        observation.append(round(joint_states_hfe, self.dec_obs))
        observation.append(round(joint_states_kfe, self.dec_obs))

        observation.append(round(base_position.x, self.dec_obs))
        observation.append(round(base_position.y, self.dec_obs))
        observation.append(round(base_position.z, self.dec_obs))  # height

        return observation

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The Monopeds height is lower than a threshhold
        2) The Orientation is outside a threshold
        """

        height_base = observations[10]

        monoped_height_ok = self.monoped_height_ok(height_base)
        monoped_orientation_ok = self.monoped_orientation_ok()

        done = not(monoped_height_ok and monoped_orientation_ok)

        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        the joint poisition, effort, contact force, orientation and distance from desired point.
        :return:
        """

        joints_state_array = observations[5:8]
        r1 = self.calculate_reward_joint_position(
            joints_state_array, self.weight_joint_position)
        # Desired Force in Newtons, taken form idle contact with 9.81 gravity.

        force_magnitude = observations[4]
        r2 = self.calculate_reward_contact_force(
            force_magnitude, self.weight_contact_force)

        rpy_array = observations[1:4]
        r3 = self.calculate_reward_orientation(
            rpy_array, self.weight_orientation)

        current_position = Point()
        current_position.x = observations[8]
        current_position.y = observations[9]
        current_position.z = observations[10]
        r4 = self.calculate_reward_distance_from_des_point(
            current_position, self.weight_distance_from_des_point)

        # The sign depend on its function.
        total_reward = self.alive_reward - r1 - r2 - r3 - r4

        rospy.logdebug("###############")
        rospy.logdebug("alive_bonus=" + str(self.alive_reward))
        rospy.logdebug("r1 joint_position=" + str(r1))
        rospy.logdebug("r2 contact_force=" + str(r2))
        rospy.logdebug("r3 orientation=" + str(r3))
        rospy.logdebug("r4 distance=" + str(r4))
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward

    # Internal TaskEnv Methods

    def is_in_desired_position(self, current_position, epsilon=0.05):
        """
        It return True if the current position is similar to the desired poistion
        """

        is_in_desired_pos = False

        x_pos_plus = self.desired_point.x + epsilon
        x_pos_minus = self.desired_point.x - epsilon
        y_pos_plus = self.desired_point.y + epsilon
        y_pos_minus = self.desired_point.y - epsilon

        x_current = current_position.x
        y_current = current_position.y

        x_pos_are_close = (x_current <= x_pos_plus) and (
            x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (
            y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        rospy.logdebug("###### IS DESIRED POS ? ######")
        rospy.logdebug("current_position"+str(current_position))
        rospy.logdebug("x_pos_plus"+str(x_pos_plus) +
                       ",x_pos_minus="+str(x_pos_minus))
        rospy.logdebug("y_pos_plus"+str(y_pos_plus) +
                       ",y_pos_minus="+str(y_pos_minus))
        rospy.logdebug("x_pos_are_close"+str(x_pos_are_close))
        rospy.logdebug("y_pos_are_close"+str(y_pos_are_close))
        rospy.logdebug("is_in_desired_pos"+str(is_in_desired_pos))
        rospy.logdebug("############")

        return is_in_desired_pos

    def is_inside_workspace(self, current_position):
        """
        Check if the monoped is inside the Workspace defined
        """
        is_inside = False

        rospy.logdebug("##### INSIDE WORK SPACE? #######")
        rospy.logdebug("XYZ current_position"+str(current_position))
        rospy.logdebug("work_space_x_max"+str(self.work_space_x_max) +
                       ",work_space_x_min="+str(self.work_space_x_min))
        rospy.logdebug("work_space_y_max"+str(self.work_space_y_max) +
                       ",work_space_y_min="+str(self.work_space_y_min))
        rospy.logdebug("work_space_z_max"+str(self.work_space_z_max) +
                       ",work_space_z_min="+str(self.work_space_z_min))
        rospy.logdebug("############")

        if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                if current_position.z > self.work_space_z_min and current_position.z <= self.work_space_z_max:
                    is_inside = True

        return is_inside

    def sonar_detected_something_too_close(self, sonar_value):
        """
        Detects if there is something too close to the monoped front
        """
        rospy.logdebug("##### SONAR TOO CLOSE? #######")
        rospy.logdebug("sonar_value"+str(sonar_value) +
                       ",min_sonar_value="+str(self.min_sonar_value))
        rospy.logdebug("############")

        too_close = sonar_value < self.min_sonar_value

        return too_close

    def monoped_has_flipped(self, current_orientation):
        """
        Based on the orientation RPY given states if the monoped has flipped
        """
        has_flipped = True

        self.max_roll = rospy.get_param("/monoped/max_roll")
        self.max_pitch = rospy.get_param("/monoped/max_pitch")

        rospy.logdebug("#### HAS FLIPPED? ########")
        rospy.logdebug("RPY current_orientation"+str(current_orientation))
        rospy.logdebug("max_roll"+str(self.max_roll) +
                       ",min_roll="+str(-1*self.max_roll))
        rospy.logdebug("max_pitch"+str(self.max_pitch) +
                       ",min_pitch="+str(-1*self.max_pitch))
        rospy.logdebug("############")

        if current_orientation.x > -1*self.max_roll and current_orientation.x <= self.max_roll:
            if current_orientation.y > -1*self.max_pitch and current_orientation.y <= self.max_pitch:
                has_flipped = False

        return has_flipped

    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)

        return distance

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def get_base_rpy(self):

        imu = self.get_imu()
        base_orientation = imu.orientation

        euler_rpy = Vector3()
        euler = euler_from_quaternion([base_orientation.x,
                                       base_orientation.y,
                                       base_orientation.z,
                                       base_orientation.w]
                                      )
        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]

        return euler_rpy

    def get_contact_force_magnitude(self):
        """
        You will see that because the X axis is the one pointing downwards, it will be the one with
        higher value when touching the floor
        For a Robot of total mas of 0.55Kg, a gravity of 9.81 m/sec**2, Weight = 0.55*9.81=5.39 N
        Falling from around 5centimetres ( negligible height ), we register peaks around
        Fx = 7.08 N
        :return:
        """
        # We get the Contact Sensor data
        lowerleg_contactsensor_state = self.get_lowerleg_contactsensor_state()
        # We extract what we need that is only the total_wrench force
        contact_force = self.get_contact_force(lowerleg_contactsensor_state)
        # We create an array with each component XYZ
        contact_force_np = numpy.array(
            (contact_force.x, contact_force.y, contact_force.z))
        # We calculate the magnitude of the Force Vector, array.
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude

    def get_contact_force(self, lowerleg_contactsensor_state):
        """
        /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
        /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

        ==> One is an array of all the forces, the other total,
         and are relative to the contact link referred to in the sensor.
        /lowerleg_contactsensor_state/states[0]/wrenches[]
        /lowerleg_contactsensor_state/states[0]/total_wrench
        :return:
        """

        # We create an empty element , in case there is no contact.
        contact_force = Vector3()
        for state in lowerleg_contactsensor_state.states:
            self.contact_force = state.total_wrench.force

        return contact_force

    def monoped_height_ok(self, height_base):

        height_ok = self.min_height <= height_base < self.max_height
        return height_ok

    def monoped_orientation_ok(self):

        orientation_rpy = self.get_base_rpy()
        roll_ok = self.max_incl_roll > abs(orientation_rpy.x)
        pitch_ok = self.max_incl_pitch > abs(orientation_rpy.y)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok

    def calculate_reward_joint_position(self, joints_state_array, weight=1.0):
        """
        We calculate reward base on the joints configuration. The more near 0 the better.
        :return:
        """
        acumulated_joint_pos = 0.0
        for joint_pos in joints_state_array:
            # Abs to remove sign influence, it doesnt matter the direction of turn.
            acumulated_joint_pos += abs(joint_pos)
            rospy.logdebug(
                "calculate_reward_joint_position>>acumulated_joint_pos=" + str(acumulated_joint_pos))
        reward = weight * acumulated_joint_pos
        rospy.logdebug(
            "calculate_reward_joint_position>>reward=" + str(reward))
        return reward

    def calculate_reward_contact_force(self, force_magnitude, weight=1.0):
        """
        We calculate reward base on the contact force.
        The nearest to the desired contact force the better.
        We use exponential to magnify big departures from the desired force.
        Default ( 7.08 N ) desired force was taken from reading of the robot touching
        the ground from a negligible height of 5cm.
        :return:
        """
        force_displacement = force_magnitude - self.desired_force

        rospy.logdebug(
            "calculate_reward_contact_force>>force_magnitude=" + str(force_magnitude))
        rospy.logdebug(
            "calculate_reward_contact_force>>force_displacement=" + str(force_displacement))
        # Abs to remove sign
        reward = weight * abs(force_displacement)
        rospy.logdebug("calculate_reward_contact_force>>reward=" + str(reward))
        return reward

    def calculate_reward_orientation(self, rpy_array, weight=1.0):
        """
        We calculate the reward based on the orientation.
        The more its closser to 0 the better because it means its upright
        desired_yaw is the yaw that we want it to be.
        to praise it to have a certain orientation, here is where to set it.
        :param: rpy_array: Its an array with Roll Pitch and Yaw in place 0, 1 and 2 respectively.
        :return:
        """

        yaw_displacement = rpy_array[2] - self.desired_yaw
        acumulated_orientation_displacement = abs(
            rpy_array[0]) + abs(rpy_array[1]) + abs(yaw_displacement)
        reward = weight * acumulated_orientation_displacement
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_reward_distance_from_des_point(self, current_position, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = self.get_distance_from_desired_point(current_position)
        reward = weight * distance
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward
