import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from openai_ros.openai_ros_common import ROSLauncher


class ParrotDroneEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new ParrotDroneEnv environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /drone/down_camera/image_raw: RGB Camera facing down.
        * /drone/front_camera/image_raw: RGB Camera facing front.
        * /drone/imu: IMU of the drone giving acceleration and orientation relative to world.
        * /drone/sonar: Sonar readings facing front
        * /drone/gt_pose: Get position and orientation in Global space
        * /drone/gt_vel: Get the linear velocity , the angular doesnt record anything.

        Actuators Topic List:
        * /cmd_vel: Move the Drone Around when you have taken off.
        * /drone/takeoff: Publish into it to take off
        * /drone/land: Publish to make ParrotDrone Land

        Args:
        """
        rospy.logdebug("Start ParrotDroneEnv INIT...")

        # Variables that we give through the constructor.
        # None in this case

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(ParrotDroneEnv, self).__init__(controllers_list=self.controllers_list,
                                             robot_name_space=self.robot_name_space,
                                             reset_controls=False,
                                             start_init_physics_parameters=False,
                                             reset_world_or_sim="WORLD")

        self.gazebo.unpauseSim()

        ROSLauncher(rospackage_name="drone_construct",
                    launch_file_name="put_robot_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/drone/down_camera/image_raw", Image,
                         self._down_camera_rgb_image_raw_callback)
        rospy.Subscriber("/drone/front_camera/image_raw", Image,
                         self._front_camera_rgb_image_raw_callback)
        rospy.Subscriber("/drone/imu", Imu, self._imu_callback)
        rospy.Subscriber("/drone/sonar", Range, self._sonar_callback)
        rospy.Subscriber("/drone/gt_pose", Pose, self._gt_pose_callback)
        rospy.Subscriber("/drone/gt_vel", Twist, self._gt_vel_callback)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._takeoff_pub = rospy.Publisher(
            '/drone/takeoff', Empty, queue_size=1)
        self._land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)

        self._check_all_publishers_ready()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished ParrotDroneEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_down_camera_rgb_image_raw_ready()
        self._check_front_camera_rgb_image_raw_ready()
        self._check_imu_ready()
        self._check_sonar_ready()
        self._check_gt_pose_ready()
        self._check_gt_vel_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_down_camera_rgb_image_raw_ready(self):
        self.down_camera_rgb_image_raw = None
        rospy.logdebug(
            "Waiting for /drone/down_camera/image_raw to be READY...")
        while self.down_camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.down_camera_rgb_image_raw = rospy.wait_for_message(
                    "/drone/down_camera/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /drone/down_camera/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /drone/down_camera/image_raw not ready yet, retrying for getting down_camera_rgb_image_raw")
        return self.down_camera_rgb_image_raw

    def _check_front_camera_rgb_image_raw_ready(self):
        self.front_camera_rgb_image_raw = None
        rospy.logdebug(
            "Waiting for /drone/front_camera/image_raw to be READY...")
        while self.front_camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.front_camera_rgb_image_raw = rospy.wait_for_message(
                    "/drone/front_camera/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /drone/front_camera/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /drone/front_camera/image_raw not ready yet, retrying for getting front_camera_rgb_image_raw")
        return self.front_camera_rgb_image_raw

    def _check_imu_ready(self):
        self.imu = None
        rospy.logdebug("Waiting for /drone/imu to be READY...")
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message(
                    "/drone/imu", Imu, timeout=5.0)
                rospy.logdebug("Current/drone/imu READY=>")

            except:
                rospy.logerr(
                    "Current /drone/imu not ready yet, retrying for getting imu")

        return self.imu

    def _check_sonar_ready(self):
        self.sonar = None
        rospy.logdebug("Waiting for /drone/sonar to be READY...")
        while self.sonar is None and not rospy.is_shutdown():
            try:
                self.sonar = rospy.wait_for_message(
                    "/drone/sonar", Range, timeout=5.0)
                rospy.logdebug("Current/drone/sonar READY=>")

            except:
                rospy.logerr(
                    "Current /drone/sonar not ready yet, retrying for getting sonar")

        return self.sonar

    def _check_gt_pose_ready(self):
        self.gt_pose = None
        rospy.logdebug("Waiting for /drone/gt_pose to be READY...")
        while self.gt_pose is None and not rospy.is_shutdown():
            try:
                self.gt_pose = rospy.wait_for_message(
                    "/drone/gt_pose", Pose, timeout=5.0)
                rospy.logdebug("Current /drone/gt_pose READY=>")

            except:
                rospy.logerr(
                    "Current /drone/gt_pose not ready yet, retrying for getting gt_pose")

        return self.gt_pose

    def _check_gt_vel_ready(self):
        self.gt_vel = None
        rospy.logdebug("Waiting for /drone/gt_vel to be READY...")
        while self.gt_vel is None and not rospy.is_shutdown():
            try:
                self.gt_vel = rospy.wait_for_message(
                    "/drone/gt_vel", Twist, timeout=5.0)
                rospy.logdebug("Current /drone/gt_vel READY=>")

            except:
                rospy.logerr(
                    "Current /drone/gt_vel not ready yet, retrying for getting gt_vel")

        return self.gt_vel

    def _down_camera_rgb_image_raw_callback(self, data):
        self.down_camera_rgb_image_raw = data

    def _front_camera_rgb_image_raw_callback(self, data):
        self.front_camera_rgb_image_raw = data

    def _imu_callback(self, data):
        self.imu = data

    def _sonar_callback(self, data):
        self.sonar = data

    def _gt_pose_callback(self, data):
        self.gt_pose = data

    def _gt_vel_callback(self, data):
        self.gt_vel = data

    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rospy.logdebug("START ALL SENSORS READY")
        self._check_cmd_vel_pub_connection()
        self._check_takeoff_pub_connection()
        self._check_land_pub_connection()
        rospy.logdebug("ALL SENSORS READY")

    def _check_cmd_vel_pub_connection(self):

        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def _check_takeoff_pub_connection(self):

        rate = rospy.Rate(10)  # 10hz
        while self._takeoff_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to _takeoff_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_takeoff_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def _check_land_pub_connection(self):

        rate = rospy.Rate(10)  # 10hz
        while self._land_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to _land_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_land_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------

    def takeoff(self):
        """
        Sends the takeoff command and checks it has taken of
        It unpauses the simulation and pauses again
        to allow it to be a self contained action
        """
        self.gazebo.unpauseSim()
        self._check_takeoff_pub_connection()

        takeoff_cmd = Empty()
        self._takeoff_pub.publish(takeoff_cmd)

        # When it takes of value of height is around 1.3
        self.wait_for_height(heigh_value_to_check=0.8,
                             smaller_than=False,
                             epsilon=0.05,
                             update_rate=10)
        self.gazebo.pauseSim()

    def land(self):
        """
        Sends the Landing command and checks it has landed
        It unpauses the simulation and pauses again
        to allow it to be a self contained action
        """
        self.gazebo.unpauseSim()

        self._check_land_pub_connection()

        land_cmd = Empty()
        self._land_pub.publish(land_cmd)
        # When Drone is on the floor, the readings are 0.5
        self.wait_for_height(heigh_value_to_check=0.6,
                             smaller_than=True,
                             epsilon=0.05,
                             update_rate=10)

        self.gazebo.pauseSim()

    def wait_for_height(self, heigh_value_to_check, smaller_than, epsilon, update_rate):
        """
        Checks if current height is smaller or bigger than a value
        :param: smaller_than: If True, we will wait until value is smaller than the one given
        """

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0

        rospy.logdebug("epsilon>>" + str(epsilon))

        while not rospy.is_shutdown():
            current_gt_pose = self._check_gt_pose_ready()

            current_height = current_gt_pose.position.z

            if smaller_than:
                takeoff_height_achieved = current_height <= heigh_value_to_check
                rospy.logwarn("SMALLER THAN HEIGHT...current_height=" +
                              str(current_height)+"<="+str(heigh_value_to_check))
            else:
                takeoff_height_achieved = current_height >= heigh_value_to_check
                rospy.logwarn("BIGGER THAN HEIGHT...current_height=" +
                              str(current_height)+">="+str(heigh_value_to_check))

            if takeoff_height_achieved:
                rospy.logwarn("Reached Height!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logwarn("Height Not there yet, keep waiting...")
            rate.sleep()

    def move_base(self, linear_speed_vector, angular_speed, epsilon=0.05, update_rate=10):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed_vector: Speed in the XYZ axis of the robot base frame, because drones can move in any direction
        :param angular_speed: Speed of the angular turning of the robot base frame, because this drone only turns on the Z axis.
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed_vector.x
        cmd_vel_value.linear.y = linear_speed_vector.y
        cmd_vel_value.linear.z = linear_speed_vector.z
        cmd_vel_value.angular.z = angular_speed
        rospy.logdebug("TurtleBot2 Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_cmd_vel_pub_connection()
        self._cmd_vel_pub.publish(cmd_vel_value)
        """
        self.wait_until_twist_achieved(cmd_vel_value,
                                        epsilon,
                                        update_rate)
        """
        self.wait_time_for_execute_movement()

    def wait_time_for_execute_movement(self):
        """
        Because this Parrot Drone position is global, we really dont have
        a way to know if its moving in the direction desired, because it would need
        to evaluate the diference in position and speed on the local reference.
        """
        time.sleep(1.0)

    def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate):
        """
        # TODO: Make it work using TF conversions
        We wait for the cmd_vel twist given to be reached by the robot reading
        from the odometry.
        :param cmd_vel_value: Twist we want to wait to reach.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        rospy.logwarn("START wait_until_twist_achieved...")

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.05

        rospy.logwarn("Desired Twist Cmd>>" + str(cmd_vel_value))
        rospy.logwarn("epsilon>>" + str(epsilon))

        values_of_ref = [cmd_vel_value.linear.x,
                         cmd_vel_value.linear.y,
                         cmd_vel_value.linear.z,
                         cmd_vel_value.angular.z]

        while not rospy.is_shutdown():
            current_gt_vel = self._check_gt_vel_ready()

            values_to_check = [current_gt_vel.linear.x,
                               current_gt_vel.linear.y,
                               current_gt_vel.linear.z,
                               current_gt_vel.angular.z]

            vel_values_are_close = self.check_array_similar(
                values_of_ref, values_to_check, epsilon)

            if vel_values_are_close:
                rospy.logwarn("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logwarn("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time)+"]")

        rospy.logwarn("END wait_until_twist_achieved...")

        return delta_time

    def check_array_similar(self, ref_value_array, check_value_array, epsilon):
        """
        It checks if the check_value id similar to the ref_value
        """
        rospy.logwarn("ref_value_array="+str(ref_value_array))
        rospy.logwarn("check_value_array="+str(check_value_array))
        return numpy.allclose(ref_value_array, check_value_array, atol=epsilon)

    def get_down_camera_rgb_image_raw(self):
        return self.down_camera_rgb_image_raw

    def get_front_camera_rgb_image_raw(self):
        return self.front_camera_rgb_image_raw

    def get_imu(self):
        return self.imu

    def get_sonar(self):
        return self.sonar

    def get_gt_pose(self):
        return self.gt_pose

    def get_gt_vel(self):
        return self.gt_vel
