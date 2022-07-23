import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import wamv_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class WamvNavTwoSetsBuoysEnv(wamv_env.WamvEnv):
    def __init__(self):
        """
        Make Wamv learn how to move straight from The starting point
        to a desired point inside the designed corridor.
        http://robotx.org/images/files/RobotX_2018_Task_Summary.pdf
        Demonstrate Navigation Control
        """

        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/wamv/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="robotx_gazebo",
                    launch_file_name="start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/wamv/config",
                               yaml_file_name="wamv_nav_twosets_buoys.yaml")

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(WamvNavTwoSetsBuoysEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here

        rospy.logdebug("Start WamvNavTwoSetsBuoysEnv INIT...")
        number_actions = rospy.get_param('/wamv/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)


        # Actions and Observations
        self.propeller_high_speed = rospy.get_param('/wamv/propeller_high_speed')
        self.propeller_low_speed = rospy.get_param('/wamv/propeller_low_speed')
        self.max_angular_speed = rospy.get_param('/wamv/max_angular_speed')
        self.max_distance_from_des_point = rospy.get_param('/wamv/max_distance_from_des_point')

        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/wamv/desired_point/x")
        self.desired_point.y = rospy.get_param("/wamv/desired_point/y")
        self.desired_point.z = rospy.get_param("/wamv/desired_point/z")
        self.desired_point_epsilon = rospy.get_param("/wamv/desired_point_epsilon")

        self.work_space_x_max = rospy.get_param("/wamv/work_space/x_max")
        self.work_space_x_min = rospy.get_param("/wamv/work_space/x_min")
        self.work_space_y_max = rospy.get_param("/wamv/work_space/y_max")
        self.work_space_y_min = rospy.get_param("/wamv/work_space/y_min")

        self.dec_obs = rospy.get_param("/wamv/number_decimals_precision_obs")


        # We place the Maximum and minimum values of observations

        high = numpy.array([self.work_space_x_max,
                            self.work_space_y_max,
                            1.57,
                            1.57,
                            3.14,
                            self.propeller_high_speed,
                            self.propeller_high_speed,
                            self.max_angular_speed,
                            self.max_distance_from_des_point
                            ])

        low = numpy.array([ self.work_space_x_min,
                            self.work_space_y_min,
                            -1*1.57,
                            -1*1.57,
                            -1*3.14,
                            -1*self.propeller_high_speed,
                            -1*self.propeller_high_speed,
                            -1*self.max_angular_speed,
                            0.0
                            ])


        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards

        self.done_reward =rospy.get_param("/wamv/done_reward")
        self.closer_to_point_reward = rospy.get_param("/wamv/closer_to_point_reward")

        self.cumulated_steps = 0.0

        rospy.logdebug("END WamvNavTwoSetsBuoysEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the two proppelers speed to 0.0 and waits for the time_sleep
        to allow the action to be executed
        """

        right_propeller_speed = 0.0
        left_propeller_speed = 0.0
        self.set_propellers_speed(  right_propeller_speed,
                                    left_propeller_speed,
                                    time_sleep=1.0)

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
        current_position = Vector3()
        current_position.x = odom.pose.pose.position.x
        current_position.y = odom.pose.pose.position.y
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(current_position)



    def _set_action(self, action):
        """
        It sets the joints of wamv based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))


        right_propeller_speed = 0.0
        left_propeller_speed = 0.0

        if action == 0: # Go Forwards
            right_propeller_speed = self.propeller_high_speed
            left_propeller_speed = self.propeller_high_speed
        elif action == 1: # Go BackWards
            right_propeller_speed = -1*self.propeller_high_speed
            left_propeller_speed = -1*self.propeller_high_speed
        elif action == 2: # Turn Left
            right_propeller_speed = self.propeller_high_speed
            left_propeller_speed = -1*self.propeller_high_speed
        elif action == 3: # Turn Right
            right_propeller_speed = -1*self.propeller_high_speed
            left_propeller_speed = self.propeller_high_speed


        # We tell wamv the propeller speeds
        self.set_propellers_speed(  right_propeller_speed,
                                    left_propeller_speed,
                                    time_sleep=1.0)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        WamvEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")

        odom = self.get_odom()
        base_position = odom.pose.pose.position
        base_orientation_quat = odom.pose.pose.orientation
        base_roll, base_pitch, base_yaw = self.get_orientation_euler(base_orientation_quat)
        base_speed_linear = odom.twist.twist.linear
        base_speed_angular_yaw = odom.twist.twist.angular.z

        distance_from_desired_point = self.get_distance_from_desired_point(base_position)

        observation = []
        observation.append(round(base_position.x,self.dec_obs))
        observation.append(round(base_position.y,self.dec_obs))

        observation.append(round(base_roll,self.dec_obs))
        observation.append(round(base_pitch,self.dec_obs))
        observation.append(round(base_yaw,self.dec_obs))

        observation.append(round(base_speed_linear.x,self.dec_obs))
        observation.append(round(base_speed_linear.y,self.dec_obs))

        observation.append(round(base_speed_angular_yaw,self.dec_obs))

        observation.append(round(distance_from_desired_point,self.dec_obs))

        return observation


    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The wamvs is ouside the workspace
        2) It got to the desired point
        """
        distance_from_desired_point = observations[8]

        current_position = Vector3()
        current_position.x = observations[0]
        current_position.y = observations[1]

        is_inside_corridor = self.is_inside_workspace(current_position)
        has_reached_des_point = self.is_in_desired_position(current_position, self.desired_point_epsilon)

        done = not(is_inside_corridor) or has_reached_des_point

        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the desired point has increased or not
        :return:
        """

        # We only consider the plane, the fluctuation in z is due mainly to wave
        current_position = Point()
        current_position.x = observations[0]
        current_position.y = observations[1]

        distance_from_des_point = self.get_distance_from_desired_point(current_position)
        distance_difference =  distance_from_des_point - self.previous_distance_from_des_point


        if not done:

            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_point_reward
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                reward = -1*self.closer_to_point_reward

        else:

            if self.is_in_desired_position(current_position, self.desired_point_epsilon):
                reward = self.done_reward
            else:
                reward = -1*self.done_reward


        self.previous_distance_from_des_point = distance_from_des_point


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods

    def is_in_desired_position(self,current_position, epsilon=0.05):
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

        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        rospy.logdebug("###### IS DESIRED POS ? ######")
        rospy.logdebug("current_position"+str(current_position))
        rospy.logdebug("x_pos_plus"+str(x_pos_plus)+",x_pos_minus="+str(x_pos_minus))
        rospy.logdebug("y_pos_plus"+str(y_pos_plus)+",y_pos_minus="+str(y_pos_minus))
        rospy.logdebug("x_pos_are_close"+str(x_pos_are_close))
        rospy.logdebug("y_pos_are_close"+str(y_pos_are_close))
        rospy.logdebug("is_in_desired_pos"+str(is_in_desired_pos))
        rospy.logdebug("############")

        return is_in_desired_pos

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

    def is_inside_workspace(self,current_position):
        """
        Check if the Wamv is inside the Workspace defined
        """
        is_inside = False

        rospy.logwarn("##### INSIDE WORK SPACE? #######")
        rospy.logwarn("XYZ current_position"+str(current_position))
        rospy.logwarn("work_space_x_max"+str(self.work_space_x_max)+",work_space_x_min="+str(self.work_space_x_min))
        rospy.logwarn("work_space_y_max"+str(self.work_space_y_max)+",work_space_y_min="+str(self.work_space_y_min))
        rospy.logwarn("############")

        if current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max:
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                    is_inside = True

        return is_inside



