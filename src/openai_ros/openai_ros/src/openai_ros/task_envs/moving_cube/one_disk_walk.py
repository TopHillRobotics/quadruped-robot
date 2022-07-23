import rospy
import numpy
import math
from gym import spaces
from openai_ros.robot_envs import cube_single_disk_env
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class MovingCubeOneDiskWalkEnv(cube_single_disk_env.CubeSingleDiskEnv):
    def __init__(self):
        
        # Launch the Task Simulated-Environment
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/moving_cube/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"
        
        ROSLauncher(rospackage_name = "moving_cube_description",
                    launch_file_name = "start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)
        
        
        
        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest( rospackage_name = "openai_ros",
                                rel_path_from_package_to_file = "src/openai_ros/task_envs/moving_cube/config",
                                yaml_file_name = "one_disk_walk.yaml")
        
        
        # Only variable needed to be set here
        number_actions = rospy.get_param('/moving_cube/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        
        
        #number_observations = rospy.get_param('/moving_cube/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """
        
        # Actions and Observations
        self.roll_speed_fixed_value = rospy.get_param('/moving_cube/roll_speed_fixed_value')
        self.roll_speed_increment_value = rospy.get_param('/moving_cube/roll_speed_increment_value')
        self.max_distance = rospy.get_param('/moving_cube/max_distance')
        max_roll = 2 * math.pi
        self.max_pitch_angle = rospy.get_param('/moving_cube/max_pitch_angle')
        self.max_y_linear_speed = rospy.get_param('/moving_cube/max_y_linear_speed')
        self.max_yaw_angle = rospy.get_param('/moving_cube/max_yaw_angle')
        
        
        high = numpy.array([
            self.roll_speed_fixed_value,
            self.max_distance,
            max_roll,
            self.max_pitch_angle,
            self.max_y_linear_speed,
            self.max_y_linear_speed,
            ])
        
        self.observation_space = spaces.Box(-high, high)
        
        rospy.logwarn("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logwarn("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Variables that we retrieve through the param server, loded when launch training launch.
        self.init_roll_vel = rospy.get_param("/moving_cube/init_roll_vel")
        
        
        # Get Observations
        self.start_point = Point()
        self.start_point.x = rospy.get_param("/moving_cube/init_cube_pose/x")
        self.start_point.y = rospy.get_param("/moving_cube/init_cube_pose/y")
        self.start_point.z = rospy.get_param("/moving_cube/init_cube_pose/z")
        
        # Rewards
        self.move_distance_reward_weight = rospy.get_param("/moving_cube/move_distance_reward_weight")
        self.y_linear_speed_reward_weight = rospy.get_param("/moving_cube/y_linear_speed_reward_weight")
        self.y_axis_angle_reward_weight = rospy.get_param("/moving_cube/y_axis_angle_reward_weight")
        self.end_episode_points = rospy.get_param("/moving_cube/end_episode_points")
        
        self.roll_reward_weight = rospy.get_param("/moving_cube/roll_reward_weight")
        self.cumulated_steps = 0.0

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(MovingCubeOneDiskWalkEnv, self).__init__(ros_ws_abspath)

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_joints(self.init_roll_vel)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.total_distance_moved = 0.0
        self.current_y_distance = self.get_y_dir_distance_from_start_point(self.start_point)
        self.pre_roll_angle = 0
        self.roll_turn_speed = rospy.get_param('/moving_cube/init_roll_vel')
        # For Info Purposes
        self.cumulated_reward = 0.0
        #self.cumulated_steps = 0.0


    def _set_action(self, action):

        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0:# Move Speed Wheel Forwards
            self.roll_turn_speed = self.roll_speed_fixed_value
        elif action == 1:# Move Speed Wheel Backwards
            self.roll_turn_speed = -1*self.roll_speed_fixed_value
        elif action == 2:# Stop Speed Wheel
            self.roll_turn_speed = 0.0
        elif action == 3:# Increment Speed
            self.roll_turn_speed += self.roll_speed_increment_value
        elif action == 4:# Decrement Speed
            self.roll_turn_speed -= self.roll_speed_increment_value

        # We clamp Values to maximum
        rospy.logdebug("roll_turn_speed before clamp=="+str(self.roll_turn_speed))
        self.roll_turn_speed = numpy.clip(self.roll_turn_speed,
                                          -1*self.roll_speed_fixed_value,
                                          self.roll_speed_fixed_value)
        rospy.logdebug("roll_turn_speed after clamp==" + str(self.roll_turn_speed))

        # We tell the OneDiskCube to spin the RollDisk at the selected speed
        self.move_joints(self.roll_turn_speed)

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        MyCubeSingleDiskEnv API DOCS
        :return:
        """

        # We get the orientation of the cube in RPY
        roll, pitch, yaw = self.get_orientation_euler()

        # We get the distance from the origin
        #distance = self.get_distance_from_start_point(self.start_point)
        y_distance = self.get_y_dir_distance_from_start_point(self.start_point)

        # We get the current speed of the Roll Disk
        current_disk_roll_vel = self.get_roll_velocity()

        # We get the linear speed in the y axis
        y_linear_speed = self.get_y_linear_speed()

        
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1)
        ]
        
        rospy.logdebug("Observations==>"+str(cube_observations))

        return cube_observations
        

    def _is_done(self, observations):

        pitch_angle = observations[3]
        yaw_angle = observations[5]

        if abs(pitch_angle) > self.max_pitch_angle:
            rospy.logerr("WRONG Cube Pitch Orientation==>" + str(pitch_angle))
            done = True
        else:
            rospy.logdebug("Cube Pitch Orientation Ok==>" + str(pitch_angle))
            if abs(yaw_angle) > self.max_yaw_angle:
                rospy.logerr("WRONG Cube Yaw Orientation==>" + str(yaw_angle))
                done = True
            else:
                rospy.logdebug("Cube Yaw Orientation Ok==>" + str(yaw_angle))
                done = False

        return done

    def _compute_reward(self, observations, done):

        if not done:

            y_distance_now = observations[1]
            delta_distance = y_distance_now - self.current_y_distance
            rospy.logdebug("y_distance_now=" + str(y_distance_now)+", current_y_distance=" + str(self.current_y_distance))
            rospy.logdebug("delta_distance=" + str(delta_distance))
            reward_distance = delta_distance * self.move_distance_reward_weight
            self.current_y_distance = y_distance_now

            y_linear_speed = observations[4]
            rospy.logdebug("y_linear_speed=" + str(y_linear_speed))
            reward_y_axis_speed = y_linear_speed * self.y_linear_speed_reward_weight

            # Negative Reward for yaw different from zero.
            yaw_angle = observations[5]
            rospy.logdebug("yaw_angle=" + str(yaw_angle))

            # Worst yaw is 90 and 270 degrees, best 0 and 180. We use sin function for giving reward.
            sin_yaw_angle = math.sin(yaw_angle)
            rospy.logdebug("sin_yaw_angle=" + str(sin_yaw_angle))
            reward_y_axis_angle = -1 * abs(sin_yaw_angle) * self.y_axis_angle_reward_weight

            #Rolling reward
            roll_angle = observations[2]
            roll_reward = math.sin(abs(self.pre_roll_angle - roll_angle)) * self.roll_reward_weight
            self.pre_roll_angle = roll_angle
            
            # We are not intereseted in decimals of the reward, doesnt give any advatage.
            reward = round(reward_distance, 0) + round(reward_y_axis_speed, 0) + round(reward_y_axis_angle, 0)
            rospy.logdebug("reward_distance=" + str(reward_distance))
            rospy.logdebug("reward_y_axis_speed=" + str(reward_y_axis_speed))
            rospy.logdebug("reward_y_axis_angle=" + str(reward_y_axis_angle))
            rospy.logdebug("reward=" + str(reward))
        else:
            reward = -1*self.end_episode_points


        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward


    # Internal TaskEnv Methods
    def get_y_dir_distance_from_start_point(self, start_point):
        """
        Calculates the distance from the given point and the current position
        given by odometry. In this case the increase or decrease in y.
        :param start_point:
        :return:
        """
        y_dist_dir = self.odom.pose.pose.position.y - start_point.y
    
        return y_dist_dir
    
    
    def get_distance_from_start_point(self, start_point):
        """
        Calculates the distance from the given point and the current position
        given by odometry
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(start_point,
                                                self.odom.pose.pose.position)
    
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
    
    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]
    
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw
    
    def get_roll_velocity(self):
        # We get the current joint roll velocity
        roll_vel = self.joints.velocity[0]
        return roll_vel
    
    def get_y_linear_speed(self):
        # We get the current joint roll velocity
        y_linear_speed = self.odom.twist.twist.linear.y
        return y_linear_speed

