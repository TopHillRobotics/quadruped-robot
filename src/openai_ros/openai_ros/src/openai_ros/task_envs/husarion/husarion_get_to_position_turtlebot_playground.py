import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import husarion_env
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class HusarionGetToPosTurtleBotPlayGroundEnv(husarion_env.HusarionEnv):
    def __init__(self):
        """
        This Task Env is designed for having the husarion in the husarion world
        closed room with columns.
        It will learn how to move around without crashing.
        """
        # Launch the Task Simulated-Environment
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/husarion/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="rosbot_gazebo",
                    launch_file_name="start_world_maze.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/husarion/config",
                               yaml_file_name="husarion_get_to_position_turtlebot_playground.yaml")

        # Only variable needed to be set here
        number_actions = rospy.get_param('/husarion/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # Actions and Observations
        self.init_linear_forward_speed = rospy.get_param(
            '/husarion/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param(
            '/husarion/init_linear_turn_speed')

        self.linear_forward_speed = rospy.get_param(
            '/husarion/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/husarion/linear_turn_speed')
        self.angular_speed = rospy.get_param('/husarion/angular_speed')

        self.new_ranges = rospy.get_param('/husarion/new_ranges')
        self.max_laser_value = rospy.get_param('/husarion/max_laser_value')
        self.min_laser_value = rospy.get_param('/husarion/min_laser_value')

        self.work_space_x_max = rospy.get_param("/husarion/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/husarion/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/husarion/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/husarion/work_space/y_min")

        # Get Desired Point to Get
        self.desired_position = Point()
        self.desired_position.x = rospy.get_param("/husarion/desired_pose/x")
        self.desired_position.y = rospy.get_param("/husarion/desired_pose/y")

        self.precision = rospy.get_param('/husarion/precision')
        self.precision_epsilon = 1.0 / (10.0 * self.precision)

        self.move_base_precision = rospy.get_param(
            '/husarion/move_base_precision')

        # We create the arrays for the laser readings
        # We also create the arrays for the odometry readings
        # We join them toeguether.

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(HusarionGetToPosTurtleBotPlayGroundEnv,
              self).__init__(ros_ws_abspath)

        laser_scan = self._check_laser_scan_ready()
        num_laser_readings = int(len(laser_scan.ranges)/self.new_ranges)
        high_laser = numpy.full((num_laser_readings), self.max_laser_value)
        low_laser = numpy.full((num_laser_readings), self.min_laser_value)

        # We place the Maximum and minimum values of the X,Y and YAW of the odometry
        # The odometry yaw can be any value in the circunference.
        high_odometry = numpy.array([self.work_space_x_max,
                                     self.work_space_y_max,
                                     3.14])
        low_odometry = numpy.array([self.work_space_x_min,
                                    self.work_space_y_min,
                                    -1*3.14])

        # Now we fetch the max and min of the Desired Position in 2D XY
        # We use the exact same as the workspace, just because make no sense
        # Consider points outside the workspace
        high_des_pos = numpy.array([self.work_space_x_max,
                                    self.work_space_y_max
                                    ])
        low_des_pos = numpy.array([self.work_space_x_min,
                                   self.work_space_y_min
                                   ])

        # We join both arrays
        high = numpy.concatenate([high_laser, high_odometry, high_des_pos])
        low = numpy.concatenate([low_laser, low_odometry, low_des_pos])

        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))

        # Rewards
        self.closer_to_point_reward = rospy.get_param(
            "/husarion/closer_to_point_reward")
        self.alive_reward = rospy.get_param("/husarion/alive_reward")
        self.end_episode_points = rospy.get_param(
            "/husarion/end_episode_points")

        self.cumulated_steps = 0.0

        self.laser_filtered_pub = rospy.Publisher(
            '/rosbot/laser/scan_filtered', LaserScan, queue_size=1)

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base(self.init_linear_forward_speed,
                       self.init_linear_turn_speed,
                       epsilon=self.move_base_precision,
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

        self.index = 0

        odometry = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(
            odometry.pose.pose.position, self.desired_position)

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
            last_action = "FORWARDS"
        elif action == 1:  # LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            last_action = "TURN_LEFT"
        elif action == 2:  # RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            last_action = "TURN_RIGHT"
        elif action == 3:  # BACKWARDS
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            last_action = "BACKWARDS"

        # We tell Husarion the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed,
                       epsilon=self.move_base_precision, update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action) +
                       ", ACTION="+str(last_action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        HusarionEnv API DOCS
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
        # We only want the X and Y position and the Yaw
        odometry_array = [round(x_position, 1),
                          round(y_position, 1),
                          round(yaw, 1)]

        # We fetch also the desired position because it conditions the learning
        # It also make it dynamic, because we can change the desired position and the
        # learning will be able to adapt.
        desired_position = [round(self.desired_position.x, 1),
                            round(self.desired_position.y, 1)]

        # We concatenate all the lists.
        observations = discretized_laser_scan + odometry_array + desired_position

        rospy.logwarn("Observations==>"+str(observations))
        rospy.logwarn("END Get Observation ==>")

        return observations

    def _is_done(self, observations):
        """
        We consider that the episode has finished when:
        1) Husarion has moved ouside the workspace defined.
        2) Husarion is too close to an object
        3) Husarion has reached the desired position
        """

        # We fetch data through the observations
        # Its all the array except from the last four elements, which are XY odom and XY des_pos
        laser_readings = observations[:-5]

        current_position = Point()
        current_position.x = observations[-5]
        current_position.y = observations[-4]
        current_position.z = 0.0

        desired_position = Point()
        desired_position.x = observations[-2]
        desired_position.y = observations[-1]
        desired_position.z = 0.0

        rospy.logwarn("is DONE? laser_readings=" + str(laser_readings))
        rospy.logwarn("is DONE? current_position=" + str(current_position))
        rospy.logwarn("is DONE? desired_position=" + str(desired_position))

        too_close_to_object = self.check_husarion_has_crashed(laser_readings)
        inside_workspace = self.check_inside_workspace(current_position)
        reached_des_pos = self.check_reached_desired_position(current_position,
                                                              desired_position,
                                                              self.precision_epsilon)

        is_done = too_close_to_object or not(
            inside_workspace) or reached_des_pos

        rospy.logwarn("####################")
        rospy.logwarn("too_close_to_object=" + str(too_close_to_object))
        rospy.logwarn("inside_workspace=" + str(inside_workspace))
        rospy.logwarn("reached_des_pos=" + str(reached_des_pos))
        rospy.logwarn("is_done=" + str(is_done))
        rospy.logwarn("######## END DONE ##")

        return is_done

    def _compute_reward(self, observations, done):
        """
        We will reward the following behaviours:
        1) The distance to the desired point has increase from last step
        2) The robot has reached the desired point

        We will penalise the following behaviours:
        1) Ending the episode without reaching the desired pos. That means it has crashed
        or it has gone outside the workspace

        """

        laser_readings = observations[:-5]

        current_position = Point()
        current_position.x = observations[-5]
        current_position.y = observations[-4]
        current_position.z = 0.0

        desired_position = Point()
        desired_position.x = observations[-2]
        desired_position.y = observations[-1]
        desired_position.z = 0.0

        distance_from_des_point = self.get_distance_from_desired_point(
            current_position, desired_position)

        distance_difference = distance_from_des_point - \
            self.previous_distance_from_des_point

        rospy.logwarn("current_position=" + str(current_position))
        rospy.logwarn("desired_point=" + str(desired_position))

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
                reward = self.alive_reward
        else:

            reached_des_pos = self.check_reached_desired_position(current_position,
                                                                  desired_position,
                                                                  self.precision_epsilon)

            if reached_des_pos:
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
    def update_desired_pos(self, new_position):
        """
        With this method you can change the desired position that you want
        Usarion to be that initialy is set through rosparams loaded through
        a yaml file possibly.
        :new_position: Type Point, because we only value the position.
        """
        self.desired_position.x = new_position.x
        self.desired_position.y = new_position.y

    def discretize_scan_observation(self, data, new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """

        discretized_ranges = []
        mod = len(data.ranges)/new_ranges

        filtered_range = []

        rospy.logdebug("data=" + str(data))
        rospy.logdebug("new_ranges=" + str(new_ranges))
        rospy.logdebug("mod=" + str(mod))

        nan_value = (self.min_laser_value + self.min_laser_value) / 2.0

        for i, item in enumerate(data.ranges):
            if (i % mod == 0):
                if item == float('Inf') or numpy.isinf(item):
                    rospy.logerr("Infinite Value=" + str(item) +
                                 "Assigning Max value")
                    discretized_ranges.append(self.max_laser_value)
                elif numpy.isnan(item):
                    rospy.logerr("Nan Value=" + str(item) +
                                 "Assigning MIN value")
                    discretized_ranges.append(self.min_laser_value)
                else:
                    # We clamp the laser readings
                    if item > self.max_laser_value:
                        rospy.logwarn("Item Bigger Than MAX, CLAMPING=>" +
                                      str(item)+", MAX="+str(self.max_laser_value))
                        discretized_ranges.append(
                            round(self.max_laser_value, 1))
                    elif item < self.min_laser_value:
                        rospy.logwarn("Item smaller Than MIN, CLAMPING=>" +
                                      str(item)+", MIN="+str(self.min_laser_value))
                        discretized_ranges.append(
                            round(self.min_laser_value, 1))
                    else:
                        rospy.logwarn(
                            "Normal Item, no processing=>" + str(item))
                        discretized_ranges.append(round(item, 1))
                # We add last value appended
                filtered_range.append(discretized_ranges[-1])
            else:
                # We add value zero
                filtered_range.append(0.0)

        rospy.logwarn(
            ">>>>>>>>>>>>>>>>>>>>>>discretized_ranges=>" + str(discretized_ranges))

        self.publish_filtered_laser_scan(laser_original_data=data,
                                         new_filtered_laser_range=filtered_range)

        return discretized_ranges

    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def get_distance_from_desired_point(self, current_position, desired_position):
        """
        Calculates the distance from the current position to the desired point
        :param current_position:
        :param desired_position:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                desired_position)

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

    def check_husarion_has_crashed(self, laser_readings):
        """
        Based on the laser readings we check if any laser readingdistance is below
        the minimum distance acceptable.
        """
        husarion_has_crashed = False

        for laser_distance in laser_readings:
            rospy.logwarn("laser_distance==>"+str(laser_distance))
            if laser_distance == self.min_laser_value:
                husarion_has_crashed = True
                rospy.logwarn("HAS CRASHED==>"+str(laser_distance) +
                              ", min="+str(self.min_laser_value))
                break

            elif laser_distance < self.min_laser_value:
                rospy.logerr("Value of laser shouldnt be lower than min==>" +
                             str(laser_distance)+", min="+str(self.min_laser_value))
            elif laser_distance > self.max_laser_value:
                rospy.logerr("Value of laser shouldnt be higher than max==>" +
                             str(laser_distance)+", max="+str(self.min_laser_value))

        return husarion_has_crashed

    def check_inside_workspace(self, current_position):
        """
        We check that the current position is inside the given workspace.
        """
        is_inside = False

        rospy.logwarn("##### INSIDE WORK SPACE? #######")
        rospy.logwarn("XYZ current_position"+str(current_position))
        rospy.logwarn("work_space_x_max"+str(self.work_space_x_max) +
                      ",work_space_x_min="+str(self.work_space_x_min))
        rospy.logwarn("work_space_y_max"+str(self.work_space_y_max) +
                      ",work_space_y_min="+str(self.work_space_y_min))
        rospy.logwarn("############")

        if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                is_inside = True

        return is_inside

    def check_reached_desired_position(self, current_position, desired_position, epsilon=0.1):
        """
        It return True if the current position is similar to the desired poistion
        """

        is_in_desired_pos = False

        x_pos_plus = desired_position.x + epsilon
        x_pos_minus = desired_position.x - epsilon
        y_pos_plus = desired_position.y + epsilon
        y_pos_minus = desired_position.y - epsilon

        x_current = current_position.x
        y_current = current_position.y

        x_pos_are_close = (x_current <= x_pos_plus) and (
            x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (
            y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        rospy.logdebug("###### IS DESIRED POS ? ######")
        rospy.logdebug("epsilon==>"+str(epsilon))
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

    def publish_filtered_laser_scan(self, laser_original_data, new_filtered_laser_range):

        length_range = len(laser_original_data.ranges)
        length_intensities = len(laser_original_data.intensities)

        laser_filtered_object = LaserScan()

        h = Header()
        # Note you need to call rospy.init_node() before this will work
        h.stamp = rospy.Time.now()
        h.frame_id = "chassis"

        laser_filtered_object.header = h
        laser_filtered_object.angle_min = laser_original_data.angle_min
        laser_filtered_object.angle_max = laser_original_data.angle_max
        laser_filtered_object.angle_increment = laser_original_data.angle_increment
        laser_filtered_object.time_increment = laser_original_data.time_increment
        laser_filtered_object.scan_time = laser_original_data.scan_time
        laser_filtered_object.range_min = laser_original_data.range_min
        laser_filtered_object.range_max = laser_original_data.range_max

        laser_filtered_object.ranges = []
        laser_filtered_object.intensities = []
        for item in new_filtered_laser_range:
            laser_filtered_object.ranges.append(item)
            laser_filtered_object.intensities.append(item)

        self.laser_filtered_pub.publish(laser_filtered_object)
