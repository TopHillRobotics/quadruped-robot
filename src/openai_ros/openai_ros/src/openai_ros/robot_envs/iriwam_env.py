import numpy
import rospy
import time
import tf
from openai_ros import robot_gazebo_env
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from control_msgs.msg import JointTrajectoryControllerState
from openai_ros.openai_ros_common import ROSLauncher
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import JointLimits


class IriWamEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all IriWamEnv environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new IriWamEnv environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /camera/depth/image_raw
        * /camera/depth/points
        * /camera/rgb/image_raw
        * /laser_scan: Laser scan of the TCP
        * /iri_wam/iri_wam_controller/state, control_msgs/JointTrajectoryControllerState: Gives desired, actual and error.

        Actuators Topic List:
        * We publish int the action: /iri_wam/iri_wam_controller/follow_joint_trajectory/goal

        Args:
        """
        rospy.logdebug("Start IriWamEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case
        # We launch the ROSlaunch that spawns the robot into the world

        ROSLauncher(rospackage_name="iri_wam_gazebo",
                    launch_file_name="put_robot_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(IriWamEnv, self).__init__(controllers_list=self.controllers_list,
                                        robot_name_space=self.robot_name_space,
                                        reset_controls=False,
                                        start_init_physics_parameters=False,
                                        reset_world_or_sim="WORLD")

        rospy.logdebug("IriWamEnv unpause...")
        self.gazebo.unpauseSim()

        self._check_all_systems_ready()

        rospy.Subscriber("/camera/depth/image_raw", Image,
                         self._camera_depth_image_raw_callback)
        rospy.Subscriber("/camera/depth/points", PointCloud2,
                         self._camera_depth_points_callback)
        rospy.Subscriber("/camera/rgb/image_raw", Image,
                         self._camera_rgb_image_raw_callback)
        rospy.Subscriber("/laser_scan", LaserScan, self._laser_scan_callback)
        rospy.Subscriber("/iri_wam/iri_wam_controller/state",
                         JointTrajectoryControllerState, self._joint_state_callback)

        self._setup_tf_listener()
        self._setup_movement_system()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished IriWamEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("IriWamEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END IriWamEnv _check_all_systems_ready...")
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        # TODO: Here go the sensors like cameras and joint states
        # self._check_camera_depth_image_raw_ready()
        # self._check_camera_depth_points_ready()
        # self._check_camera_rgb_image_raw_ready()
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_camera_depth_image_raw_ready(self):
        self.camera_depth_image_raw = None
        rospy.logdebug("Waiting for /camera/depth/image_raw to be READY...")
        while self.camera_depth_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_depth_image_raw = rospy.wait_for_message(
                    "/camera/depth/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /camera/depth/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /camera/depth/image_raw not ready yet, retrying for getting camera_depth_image_raw")
        return self.camera_depth_image_raw

    def _check_camera_depth_points_ready(self):
        self.camera_depth_points = None
        rospy.logdebug("Waiting for /camera/depth/points to be READY...")
        while self.camera_depth_points is None and not rospy.is_shutdown():
            try:
                self.camera_depth_points = rospy.wait_for_message(
                    "/camera/depth/points", PointCloud2, timeout=10.0)
                rospy.logdebug("Current /camera/depth/points READY=>")

            except:
                rospy.logerr(
                    "Current /camera/depth/points not ready yet, retrying for getting camera_depth_points")
        return self.camera_depth_points

    def _check_camera_rgb_image_raw_ready(self):
        self.camera_rgb_image_raw = None
        rospy.logdebug("Waiting for /camera/rgb/image_raw to be READY...")
        while self.camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_rgb_image_raw = rospy.wait_for_message(
                    "/camera/rgb/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /camera/rgb/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /camera/rgb/image_raw not ready yet, retrying for getting camera_rgb_image_raw")
        return self.camera_rgb_image_raw

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        rospy.logdebug("Waiting for /laser_scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message(
                    "/laser_scan", LaserScan, timeout=5.0)
                rospy.logdebug("Current /laser_scan READY=>")

            except:
                rospy.logerr(
                    "Current /laser_scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan

    def _check_joint_state_ready(self):
        self.joint_state = None
        rospy.logdebug(
            "Waiting for /iri_wam/iri_wam_controller/state to be READY...")
        while self.joint_state is None and not rospy.is_shutdown():
            try:
                self.joint_state = rospy.wait_for_message(
                    "/iri_wam/iri_wam_controller/state", JointTrajectoryControllerState, timeout=5.0)
                rospy.logdebug(
                    "Current /iri_wam/iri_wam_controller/state READY=>")

            except:
                rospy.logerr(
                    "Current /iri_wam/iri_wam_controller/state not ready yet, retrying for getting laser_scan")
        return self.joint_state

    def _camera_depth_image_raw_callback(self, data):
        self.camera_depth_image_raw = data

    def _camera_depth_points_callback(self, data):
        self.camera_depth_points = data

    def _camera_rgb_image_raw_callback(self, data):
        self.camera_rgb_image_raw = data

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _joint_state_callback(self, data):
        self.joint_state = data

    def _setup_tf_listener(self):
        """
        Set ups the TF listener for getting the transforms you ask for.
        """
        self.listener = tf.TransformListener()

    def _setup_movement_system(self):
        """
        Setup of the movement system.
        :return:
        """
        self.traj_object = IriWamExecTrajectory()

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
    def move_joints_to_angle_blocking(self, joints_positions_array):
        """
        It moves all the joints to the given position and doesnt exit until it reaches that position
        :param: joints_positions_array: Its an array that ahas the desired joint positions in radians. The order of the
        joints is:
            [   "iri_wam_joint_1",
                "iri_wam_joint_2",
                "iri_wam_joint_3",
                "iri_wam_joint_4",
                "iri_wam_joint_5",
                "iri_wam_joint_6",
                "iri_wam_joint_7"]

        """
        self.traj_object.send_joints_positions(joints_positions_array)

    def get_tf_start_to_end_frames(self, start_frame_name, end_frame_name):
        """
        Given two frames, it returns the transform from the start_frame_name to the end_frame_name.
        It will only return something different to None if the TFs of the Two frames are in TF topic
        published and are connected through the TF tree.
        :param: start_frame_name: Start Frame of the TF transform
                end_frame_name: End Frame of the TF transform
        :return: trans,rot of the transform between the start and end frames.
        """
        start_frame = "/"+start_frame_name
        end_frame = "/"+end_frame_name

        trans, rot = None, None

        try:
            (trans, rot) = self.listener.lookupTransform(
                start_frame, end_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF start to end not ready YET...")
            pass

        return trans, rot

    def get_camera_depth_image_raw(self):
        return self.camera_depth_image_raw

    def get_camera_depth_points(self):
        return self.camera_depth_points

    def get_camera_rgb_image_raw(self):
        return self.camera_rgb_image_raw

    def get_laser_scan(self):
        return self.laser_scan

    def get_joint_state(self):
        return self.joint_state

    def get_joint_limits(self):
        """
        name: [iri_wam_joint_1, iri_wam_joint_2, iri_wam_joint_3, iri_wam_joint_4, iri_wam_joint_5,
  iri_wam_joint_6, iri_wam_joint_7]
position: [-2.453681702263566e-11, 5.5375411835534294e-05, -2.9760194308892096e-11, -0.0062733383258359865, 1.8740564655672642e-13, 2.6570746959997393e-05, 1.5187850976872141e-13]
        """

        name_array = ["iri_wam_joint_1",
                      "iri_wam_joint_2",
                      "iri_wam_joint_3",
                      "iri_wam_joint_4",
                      "iri_wam_joint_5",
                      "iri_wam_joint_6",
                      "iri_wam_joint_7"]

        up_limits_array = [2.6,
                           2.0,
                           2.8,
                           3.1,
                           1.24,
                           1.6,
                           3.0]

        down_limits_array = [-2.6,
                             -2.0,
                             -2.8,
                             -0.9,
                             -4.76,
                             -1.6,
                             -3.0]

        joint_limits_array = []
        for i in range(len(name_array)):
            joint_limits = JointLimits()
            joint_limits.min_position = down_limits_array[i]
            joint_limits.max_position = up_limits_array[i]
            joint_limits_array.append(joint_limits)

        return joint_limits_array

    def init_joint_limits(self):
        """
        Get the Joint Limits, in the init fase where we need to unpause the simulation to get them
        :return: joint_limits: The Joint Limits Dictionary, with names, angles, vel and effort limits.
        """
        joint_limits = self.get_joint_limits()
        return joint_limits


class IriWamExecTrajectory(object):

    def __init__(self):

        # create the connection to the action server
        self.client = actionlib.SimpleActionClient(
            '/iri_wam/iri_wam_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # waits until the action server is up and running
        self.client.wait_for_server()

        self.init_goal_message()

    def init_goal_message(self):
        """
        We initialise the variable that we will use to send the goals.
        We will reuse it because most of the values are fixed.
        """

        self.PENDING = 0
        self.ACTIVE = 1
        self.DONE = 2
        self.WARN = 3
        self.ERROR = 4

        # We Initialise the GOAL SYETS GOINT TO INIT POSE
        # creates a goal to send to the action server
        self.goal = FollowJointTrajectoryGoal()

        # We fill in the Goal

        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.goal.trajectory.header.frame_id = "iri_wam_link_base"
        self.goal.trajectory.joint_names = ["iri_wam_joint_1",
                                            "iri_wam_joint_2",
                                            "iri_wam_joint_3",
                                            "iri_wam_joint_4",
                                            "iri_wam_joint_5",
                                            "iri_wam_joint_6",
                                            "iri_wam_joint_7"]

        # Some of them dont quite coincide with the URDF limits, just because those limits break the simulation.
        self.max_values = [2.6,
                           2.0,
                           2.8,
                           3.0,
                           1.24,
                           1.5,
                           3.0]

        self.min_values = [-2.6,
                           -2.0,
                           -2.8,
                           -0.9,
                           -1.24,
                           -1.4,
                           -3.0]

        self.goal.trajectory.points = []
        joint_traj_point = JointTrajectoryPoint()

        # TODO
        joint_traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.effort = []
        joint_traj_point.time_from_start = rospy.Duration.from_sec(1.0)

        self.goal.trajectory.points.append(joint_traj_point)

    def get_goal(self):
        return self.goal

    def feedback_callback(self, feedback):

        rospy.loginfo("##### FEEDBACK ######")
        # rospy.loginfo(str(feedback.joint_names))
        # rospy.loginfo(str(feedback.desired.positions))
        # rospy.loginfo(str(feedback.actual.positions))
        rospy.loginfo(str(feedback.error.positions))
        rospy.loginfo("##### ###### ######")

    def send_joints_positions(self, joints_positions_array, seconds_duration=0.05):
        rospy.logdebug("send_joints_positions: "+str(joints_positions_array))
        my_goal = self.get_goal()

        my_goal.trajectory.header.stamp = rospy.Time.now()
        joint_traj_point = JointTrajectoryPoint()

        # We clamp the values to max and min to avoid asking configurations that IriWam cant reach.

        joint_traj_point.positions = numpy.clip(joints_positions_array,
                                                self.min_values,
                                                self.max_values).tolist()
        joint_traj_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.effort = []
        joint_traj_point.time_from_start = rospy.Duration.from_sec(
            seconds_duration)

        my_goal.trajectory.points = []
        my_goal.trajectory.points.append(joint_traj_point)

        # sends the goal to the action server, specifying which feedback function
        # to call when feedback received
        self.client.send_goal(my_goal, feedback_cb=self.feedback_callback)

        # Uncomment these lines to test goal preemption:
        # self.client.cancel_goal()  # would cancel the goal 3 seconds after starting

        # state_result = self.client.get_state()

        # rate = rospy.Rate(10)

        # rospy.loginfo("state_result: "+str(state_result))

        # while state_result < self.DONE:
        #     rospy.loginfo(
        #         "Doing Stuff while waiting for the Server to give a result....")
        #     rate.sleep()
        #     state_result = self.client.get_state()
        #     rospy.loginfo("state_result: "+str(state_result))

        # rospy.loginfo("[Result] State: "+str(state_result))
        # if state_result == self.ERROR:
        #     rospy.logerr("Something went wrong in the Server Side")
        # if state_result == self.WARN:
        #     rospy.logdebug("There is a warning in the Server Side")
