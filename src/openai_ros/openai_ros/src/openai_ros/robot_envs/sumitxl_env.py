import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from openai_ros.openai_ros_common import ROSLauncher


class SumitXlEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new SumitXlEnv environment.

        Execute a call to service /summit_xl/controller_manager/list_controllers
        To get the list of controllers to be restrarted

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /gps/fix : GPS position Data
        * /gps/fix_velocity: GPS Speed data
        * /hokuyo_base/scan: Laser Readings
        * /imu/data: Inertial Mesurment Unit data, orientation and acceleration
        * /orbbec_astra/depth/image_raw
        * /orbbec_astra/depth/points
        * /orbbec_astra/rgb/image_raw
        * /summit_xl/odom: Odometry

        Actuators Topic List: /cmd_vel,

        Args:
        """
        print("Start SumitXlEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="summit_xl_gazebo",
                    launch_file_name="put_robot_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)
        print("SPAWN DONE SumitXlEnv INIT...")
        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = ["joint_read_state_controller",
                                 "joint_blw_velocity_controller",
                                 "joint_brw_velocity_controller",
                                 "joint_flw_velocity_controller",
                                 "joint_frw_velocity_controller"
                                 ]

        # It doesnt use namespace
        self.robot_name_space = "summit_xl"

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        print("START OpenAIROS CORE SumitXlEnv INIT...")
        super(SumitXlEnv, self).__init__(controllers_list=self.controllers_list,
                                         robot_name_space=self.robot_name_space,
                                         reset_controls=False,
                                         start_init_physics_parameters=False,
                                         reset_world_or_sim="WORLD")
        print("DONE OpenAIROS CORE SumitXlEnv INIT...")
        self.gazebo.unpauseSim()
        # TODO: See why this doesnt work in Summit XL
        # self.controllers_object.reset_controllers()

        print("START CHECK SENSORS SumitXlEnv INIT...")
        self._check_all_sensors_ready()
        print("DONE CHECK SENSORS SumitXlEnv INIT...")

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/gps/fix", NavSatFix, self._gps_fix_callback)
        rospy.Subscriber("/gps/fix_velocity", Vector3Stamped,
                         self._gps_fix_velocity_callback)

        rospy.Subscriber("/orbbec_astra/depth/image_raw", Image,
                         self._camera_depth_image_raw_callback)
        rospy.Subscriber("/orbbec_astra/depth/points",
                         PointCloud2, self._camera_depth_points_callback)
        rospy.Subscriber("/orbbec_astra/rgb/image_raw", Image,
                         self._camera_rgb_image_raw_callback)

        rospy.Subscriber("/hokuyo_base/scan", LaserScan,
                         self._laser_scan_callback)
        rospy.Subscriber("/imu/data", Imu, self._imu_callback)
        rospy.Subscriber("/summit_xl/odom", Odometry, self._odom_callback)

        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        print("START CHECK PUBLISHERS SumitXlEnv INIT...")
        self._check_publishers_connection()
        print("DONE CHECK PUBLISHERS SumitXlEnv INIT...")

        self.gazebo.pauseSim()

        print("Finished SumitXlEnv INIT...")

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
        print("START ALL SENSORS READY")
        self._check_gps_fix_ready()
        self._check_gps_fix_velocity_ready()
        self._check_camera_depth_image_raw_ready()
        self._check_camera_depth_points_ready()
        self._check_camera_rgb_image_raw_ready()
        self._check_odom_ready()
        self._check_imu_ready()
        self._check_laser_scan_ready()
        print("ALL SENSORS READY")

    def _check_gps_fix_ready(self):
        self.gps_fix = None
        print("Waiting for /gps/fix to be READY...")
        while self.gps_fix is None and not rospy.is_shutdown():
            try:
                self.gps_fix = rospy.wait_for_message(
                    "/gps/fix", NavSatFix, timeout=5.0)
                print("Current /gps/fix READY=>")

            except:
                rospy.logerr(
                    "Current /gps/fix not ready yet, retrying for getting odom")

        return self.gps_fix

    def _check_gps_fix_velocity_ready(self):
        self.gps_fix_velocity = None
        print("Waiting for /gps/fix_velocity to be READY...")
        while self.gps_fix_velocity is None and not rospy.is_shutdown():
            try:
                self.gps_fix_velocity = rospy.wait_for_message(
                    "/gps/fix_velocity", Vector3Stamped, timeout=5.0)
                print("Current /gps/fix_velocity READY=>")

            except:
                rospy.logerr(
                    "Current /gps/fix_velocity not ready yet, retrying for getting odom")

        return self.gps_fix_velocity

    def _check_camera_depth_image_raw_ready(self):
        self.camera_depth_image_raw = None
        print("Waiting for /orbbec_astra/depth/image_raw to be READY...")
        while self.camera_depth_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_depth_image_raw = rospy.wait_for_message(
                    "/orbbec_astra/depth/image_raw", Image, timeout=5.0)
                print("Current /orbbec_astra/depth/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /orbbec_astra/depth/image_raw not ready yet, retrying for getting camera_depth_image_raw")
        return self.camera_depth_image_raw

    def _check_camera_depth_points_ready(self):
        self.camera_depth_points = None
        print("Waiting for /orbbec_astra/depth/points to be READY...")
        while self.camera_depth_points is None and not rospy.is_shutdown():
            try:
                self.camera_depth_points = rospy.wait_for_message(
                    "/orbbec_astra/depth/points", PointCloud2, timeout=10.0)
                print("Current /orbbec_astra/depth/points READY=>")

            except:
                rospy.logerr(
                    "Current /orbbec_astra/depth/points not ready yet, retrying for getting camera_depth_points")
        return self.camera_depth_points

    def _check_camera_rgb_image_raw_ready(self):
        self.camera_rgb_image_raw = None
        print("Waiting for /orbbec_astra/rgb/image_raw to be READY...")
        while self.camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_rgb_image_raw = rospy.wait_for_message(
                    "/orbbec_astra/rgb/image_raw", Image, timeout=5.0)
                print("Current /orbbec_astra/rgb/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /orbbec_astra/rgb/image_raw not ready yet, retrying for getting camera_rgb_image_raw")
        return self.camera_rgb_image_raw

    def _check_odom_ready(self):
        self.odom = None
        print("Waiting for /summit_xl/odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message(
                    "/summit_xl/odom", Odometry, timeout=0.5)
                print("Current /summit_xl/odom READY=>")

            except:
                rospy.logerr(
                    "Current /summit_xl/odom not ready yet, retrying for getting odom")

        return self.odom

    def _check_imu_ready(self):
        self.imu = None
        print("Waiting for /imu/data to be READY...")
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message(
                    "/imu/data", Imu, timeout=5.0)
                print("Current /imu/data READY=>")

            except:
                rospy.logerr(
                    "Current /imu/data not ready yet, retrying for getting imu")

        return self.imu

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        print("Waiting for /hokuyo_base/scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message(
                    "/hokuyo_base/scan", LaserScan, timeout=1.0)
                print("Current /hokuyo_base/scan READY=>")

            except:
                rospy.logerr(
                    "Current /hokuyo_base/scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan

    def _gps_fix_callback(self, data):
        self.gps_fix = data

    def _gps_fix_velocity_callback(self, data):
        self.gps_fix_velocity = data

    def _camera_depth_image_raw_callback(self, data):
        self.camera_depth_image_raw = data

    def _camera_depth_points_callback(self, data):
        self.camera_depth_points = data

    def _camera_rgb_image_raw_callback(self, data):
        self.camera_rgb_image_raw = data

    def _odom_callback(self, data):
        self.odom = data

    def _imu_callback(self, data):
        self.imu = data

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            print("No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        print("_cmd_vel_pub Publisher Connected")

        print("All Publishers READY")

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
    def move_base(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed: Speed in the X axis of the robot base frame
        :param angular_speed: Speed of the angular turning of the robot base frame
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        print("SumitXL Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_publishers_connection()
        self._cmd_vel_pub.publish(cmd_vel_value)
        time.sleep(0.2)
        """
        self.wait_until_twist_achieved(cmd_vel_value,
                                       epsilon,
                                       update_rate)
        """

    def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate):
        """
        We wait for the cmd_vel twist given to be reached by the robot reading
        from the odometry.
        :param cmd_vel_value: Twist we want to wait to reach.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        print("START wait_until_twist_achieved...")

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0

        print("Desired Twist Cmd>>" + str(cmd_vel_value))
        print("epsilon>>" + str(epsilon))

        linear_speed = cmd_vel_value.linear.x
        angular_speed = cmd_vel_value.angular.z

        linear_speed_plus = linear_speed + epsilon
        linear_speed_minus = linear_speed - epsilon
        # Correcting factor for angular based on observations
        angular_factor = 2.0
        epsilon_angular_factor = 6.0
        angular_speed_plus = (angular_factor * angular_speed) + \
            (epsilon * epsilon_angular_factor)
        angular_speed_minus = (angular_factor * angular_speed) - \
            (epsilon * epsilon_angular_factor)

        while not rospy.is_shutdown():
            current_odometry = self._check_odom_ready()

            odom_linear_vel = current_odometry.twist.twist.linear.x
            """
            When asking to turn EX: angular.Z = 0.3 --> Odometry is 0.6
            In linera runs ok. It also flutuates a lot, due to the turning through friction.
            Therefore we will have to multiply the angular by 2 and broaden the
            accepted error for angular.
            """
            odom_angular_vel = current_odometry.twist.twist.angular.z

            print("Linear VEL=" + str(odom_linear_vel) +
                  ", ?RANGE=[" + str(linear_speed_minus) + ","+str(linear_speed_plus)+"]")
            print("Angular VEL=" + str(odom_angular_vel) +
                  ", ?RANGE=[" + str(angular_speed_minus) + ","+str(angular_speed_plus)+"]")

            linear_vel_are_close = (odom_linear_vel <= linear_speed_plus) and (
                odom_linear_vel > linear_speed_minus)
            angular_vel_are_close = (odom_angular_vel <= angular_speed_plus) and (
                odom_angular_vel > angular_speed_minus)

            if linear_vel_are_close and angular_vel_are_close:
                print("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            print("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        print("[Wait Time=" + str(delta_time)+"]")

        print("END wait_until_twist_achieved...")

        return delta_time

    def get_gps_fix(self):
        return self.gps_fix

    def get_gps_fix_velocity(self):
        return self.gps_fix_velocity

    def get_laser_scan(self):
        return self.laser_scan

    def get_camera_depth_image_raw(self):
        return self.camera_depth_image_raw

    def get_camera_depth_points(self):
        return self.camera_depth_points

    def get_camera_rgb_image_raw(self):
        return self.camera_rgb_image_raw

    def get_odom(self):
        return self.odom

    def get_imu(self):
        return self.imu
