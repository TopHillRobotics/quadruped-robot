import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import sumitxl_env
from gym.envs.registration import register
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class SumitXlRoom(sumitxl_env.SumitXlEnv):
    def __init__(self):
        """
        This Task Env is designed for having the sumit_xl in the sumit_xl world
        closed room with columns.
        It will learn how to move around without crashing.
        """

        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/sumit_xl/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="summit_xl_gazebo",
                    launch_file_name="start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/sumit_xl/config",
                               yaml_file_name="sumit_xl_room.yaml")

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(SumitXlRoom, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        number_actions = rospy.get_param('/sumit_xl/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param(
            '/sumit_xl/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/sumit_xl/linear_turn_speed')
        self.angular_speed = rospy.get_param('/sumit_xl/angular_speed')
        self.init_linear_forward_speed = rospy.get_param(
            '/sumit_xl/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param(
            '/sumit_xl/init_linear_turn_speed')

        self.new_ranges = rospy.get_param('/sumit_xl/new_ranges')
        self.min_range = rospy.get_param('/sumit_xl/min_range')
        self.max_laser_value = rospy.get_param('/sumit_xl/max_laser_value')
        self.min_laser_value = rospy.get_param('/sumit_xl/min_laser_value')
        self.max_linear_aceleration = rospy.get_param(
            '/sumit_xl/max_linear_aceleration')

        self.max_distance = rospy.get_param('/sumit_xl/max_distance')

        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/sumit_xl/desired_pose/x")
        self.desired_point.y = rospy.get_param("/sumit_xl/desired_pose/y")
        self.desired_point.z = rospy.get_param("/sumit_xl/desired_pose/z")

        # We create the arrays for the laser readings
        # We also create the arrays for the odometry readings
        # We join them toeguether.
        laser_scan = self.get_laser_scan()

        num_laser_readings = int(len(laser_scan.ranges)/self.new_ranges)

        high_laser = numpy.full((num_laser_readings), self.max_laser_value)
        low_laser = numpy.full((num_laser_readings), self.min_laser_value)

        # We place the Maximum and minimum values of the X,Y and YAW of the odometry
        # The odometry yaw can be any value in the circunference.
        high_odometry = numpy.array(
            [self.max_distance, self.max_distance, 3.14])
        low_odometry = numpy.array(
            [-1*self.max_distance, -1*self.max_distance, -1*3.14])

        # We join both arrays
        high = numpy.concatenate([high_laser, high_odometry])
        low = numpy.concatenate([low_laser, low_odometry])

        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))

        # Rewards
        self.closer_to_point_reward = rospy.get_param(
            "/sumit_xl/closer_to_point_reward")
        self.not_ending_point_reward = rospy.get_param(
            "/sumit_xl/not_ending_point_reward")

        self.end_episode_points = rospy.get_param(
            "/sumit_xl/end_episode_points")

        self.cumulated_steps = 0.0

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base(self.init_linear_forward_speed,
                       self.init_linear_turn_speed,
                       epsilon=0.05,
                       update_rate=10)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False

        odometry = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(
            odometry.pose.pose.position)

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the SumitXl
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0:  # FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1:  # LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2:  # RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"
        """
        elif action == 3: #STOP
            linear_speed = 0.0
            angular_speed = 0.0
            self.last_action = "STOP"
        """

        # We tell SumitXL the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed,
                       epsilon=0.05, update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        SumitXlEnv API DOCS

        WALL CLOSE LEFT [1, 1, 9, 0, 0, 0, -1.8, 0.46, 0.01]
        WALL CLOSE RIGHT [0, 0, 0, 10, 1, 2, -1.8, -0.61, 0.01]
        WALL BACK [0, 9, 1, 1, 6, 0, -1.8, -0.54, 1.59]
        WALL FRONT [2, 9, 0, 0, 2, 2, -1.83, 0.51, 1.58]

        0 in reality is arround front 0.4, back 0.5, sides 0.3

        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()

        discretized_laser_scan = self.discretize_scan_observation(laser_scan,
                                                                  self.new_ranges
                                                                  )
        # We get the odometry so that SumitXL knows where it is.
        odometry = self.get_odom()
        x_position = odometry.pose.pose.position.x
        y_position = odometry.pose.pose.position.y

        # We get the orientation of the cube in RPY
        roll, pitch, yaw = self.get_orientation_euler()
        # We round to only two decimals to avoid very big Observation space
        odometry_array = [round(x_position, 2),
                          round(y_position, 2),
                          round(yaw, 2)]

        # We only want the X and Y position and the Yaw

        observations = discretized_laser_scan + odometry_array

        rospy.logdebug("Observations==>"+str(observations))
        rospy.logdebug("END Get Observation ==>")

        return observations

    def _is_done(self, observations):

        if self._episode_done:
            rospy.logerr("SumitXl is Too Close to wall==>")
        else:
            rospy.logdebug("SumitXl is NOT close to a wall ==>")

        # Now we check if it has crashed based on the imu
        imu_data = self.get_imu()
        linear_acceleration_magnitude = self.get_vector_magnitude(
            imu_data.linear_acceleration)
        if linear_acceleration_magnitude > self.max_linear_aceleration:
            rospy.logerr("SumitXl Crashed==>"+str(linear_acceleration_magnitude) +
                         ">"+str(self.max_linear_aceleration))
            self._episode_done = True
        else:
            rospy.logerr("DIDNT crash SumitXl ==>"+str(linear_acceleration_magnitude) +
                         ">"+str(self.max_linear_aceleration))

        current_position = Point()
        current_position.x = observations[-3]
        current_position.y = observations[-2]
        current_position.z = 0.0

        if abs(current_position.x) <= self.max_distance:
            if abs(current_position.y) <= self.max_distance:
                rospy.logdebug(
                    "SummitXL Position is OK ==>["+str(current_position.x)+","+str(current_position.y)+"]")
            else:
                rospy.logerr("SummitXL to Far in Y Pos ==>" +
                             str(current_position.x))
                self._episode_done = True
        else:
            rospy.logerr("SummitXL to Far in X Pos ==>" +
                         str(current_position.x))
            self._episode_done = True

        if self.is_in_desired_position(current_position):
            self._episode_done = True

        return self._episode_done

    def _compute_reward(self, observations, done):
        """
        We give reward to the robot when it gets closer to the desired point.
        We Dont give it contsnatly, but only if there is an improvement
        """

        # We get the current Position from the obervations
        current_position = Point()
        current_position.x = observations[-3]
        current_position.y = observations[-2]
        current_position.z = 0.0

        distance_from_des_point = self.get_distance_from_desired_point(
            current_position)

        distance_difference = distance_from_des_point - \
            self.previous_distance_from_des_point

        rospy.logwarn("current_position=" + str(current_position))
        rospy.logwarn("desired_point=" + str(self.desired_point))

        rospy.logwarn("total_distance_from_des_point=" +
                      str(self.previous_distance_from_des_point))
        rospy.logwarn("distance_from_des_point=" +
                      str(distance_from_des_point))
        rospy.logwarn("distance_difference=" + str(distance_difference))

        if not done:
            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_point_reward
            else:
                # If it didnt get closer, we give much less points in theory
                # This should trigger the behaviour of moving towards the point
                rospy.logwarn("NO DECREASE IN DISTANCE, so much less points")
                reward = self.not_ending_point_reward
        else:
            if self.is_in_desired_position(current_position):
                reward = self.end_episode_points
                rospy.logwarn(
                    "GOT TO DESIRED POINT ; DONE, reward=" + str(reward))
            else:
                reward = -1*self.end_episode_points
                rospy.logerr(
                    "SOMETHING WENT WRONG ; DONE, reward=" + str(reward))

        self.previous_distance_from_des_point = distance_from_des_point

        rospy.logwarn("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward

    # Internal TaskEnv Methods

    def discretize_scan_observation(self, data, new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False

        discretized_ranges = []
        mod = len(data.ranges)/new_ranges

        rospy.logdebug("data=" + str(data))
        rospy.logdebug("new_ranges=" + str(new_ranges))
        rospy.logdebug("mod=" + str(mod))

        for i, item in enumerate(data.ranges):
            if (i % mod == 0):
                if item == float('Inf') or numpy.isinf(item):
                    discretized_ranges.append(self.max_laser_value)
                elif numpy.isnan(item):
                    discretized_ranges.append(self.min_laser_value)
                else:
                    discretized_ranges.append(int(item))

                if (self.min_range > item > 0):
                    rospy.logerr("done Validation >>> item=" +
                                 str(item)+"< "+str(self.min_range))
                    self._episode_done = True
                else:
                    rospy.logdebug("NOT done Validation >>> item=" +
                                   str(item)+"< "+str(self.min_range))

        return discretized_ranges

    def get_vector_magnitude(self, vector):
        """
        It calculated the magnitude of the Vector3 given.
        This is usefull for reading imu accelerations and knowing if there has been
        a crash
        :return:
        """
        contact_force_np = numpy.array((vector.x, vector.y, vector.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude

    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

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

        return is_in_desired_pos
