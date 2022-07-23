import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from openai_ros.openai_ros_common import ROSLauncher


class HopperEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all HopperEnv environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new HopperEnv environment.

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
        rospy.logdebug("Start HopperEnv INIT...")

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="legged_robots_sims",
                    launch_file_name="put_robot_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Variables that we give through the constructor.
        # None in this case

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = ['joint_state_controller',
                                 'haa_joint_position_controller',
                                 'hfe_joint_position_controller',
                                 'kfe_joint_position_controller']

        # It doesnt use namespace
        self.robot_name_space = "monoped"

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(HopperEnv, self).__init__(controllers_list=self.controllers_list,
                                        robot_name_space=self.robot_name_space,
                                        reset_controls=False,
                                        start_init_physics_parameters=False,
                                        reset_world_or_sim="WORLD")

        rospy.logdebug("HopperEnv unpause1...")
        self.gazebo.unpauseSim()
        # self.controllers_object.reset_controllers()

        self._check_all_systems_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/monoped/imu/data", Imu, self._imu_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/lowerleg_contactsensor_state",
                         ContactsState, self._contact_callback)
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/monoped/joint_states", JointState,
                         self._joints_state_callback)

        self.publishers_array = []
        self._haa_joint_pub = rospy.Publisher(
            '/monoped/haa_joint_position_controller/command', Float64, queue_size=1)
        self._hfe_joint_pub = rospy.Publisher(
            '/monoped/hfe_joint_position_controller/command', Float64, queue_size=1)
        self._kfe_joint_pub = rospy.Publisher(
            '/monoped/kfe_joint_position_controller/command', Float64, queue_size=1)

        self.publishers_array.append(self._haa_joint_pub)
        self.publishers_array.append(self._hfe_joint_pub)
        self.publishers_array.append(self._kfe_joint_pub)

        self._check_all_publishers_ready()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished HopperEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("HopperEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END HopperEnv _check_all_systems_ready...")
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        self._check_imu_ready()
        self._check_lowerleg_contactsensor_state_ready()
        self._check_joint_states_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message(
                    "/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /odom READY=>")

            except:
                rospy.logerr(
                    "Current /odom not ready yet, retrying for getting odom")
        return self.odom

    def _check_imu_ready(self):
        self.imu = None
        rospy.logdebug("Waiting for /monoped/imu/data to be READY...")
        while self.imu is None and not rospy.is_shutdown():
            try:
                self.imu = rospy.wait_for_message(
                    "/monoped/imu/data", Imu, timeout=1.0)
                rospy.logdebug("Current /monoped/imu/data READY=>")

            except:
                rospy.logerr(
                    "Current /monoped/imu/data not ready yet, retrying for getting imu")
        return self.imu

    def _check_lowerleg_contactsensor_state_ready(self):
        self.lowerleg_contactsensor_state = None
        rospy.logdebug(
            "Waiting for /lowerleg_contactsensor_state to be READY...")
        while self.lowerleg_contactsensor_state is None and not rospy.is_shutdown():
            try:
                self.lowerleg_contactsensor_state = rospy.wait_for_message(
                    "/lowerleg_contactsensor_state", ContactsState, timeout=1.0)
                rospy.logdebug("Current /lowerleg_contactsensor_state READY=>")

            except:
                rospy.logerr(
                    "Current /lowerleg_contactsensor_state not ready yet, retrying for getting lowerleg_contactsensor_state")
        return self.lowerleg_contactsensor_state

    def _check_joint_states_ready(self):
        self.joint_states = None
        rospy.logdebug("Waiting for /monoped/joint_states to be READY...")
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(
                    "/monoped/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current /monoped/joint_states READY=>")

            except:
                rospy.logerr(
                    "Current /monoped/joint_states not ready yet, retrying for getting joint_states")
        return self.joint_states

    def _odom_callback(self, data):
        self.odom = data

    def _imu_callback(self, data):
        self.imu = data

    def _contact_callback(self, data):
        self.lowerleg_contactsensor_state = data

    def _joints_state_callback(self, data):
        self.joint_states = data

    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rospy.logdebug("START ALL SENSORS READY")
        for publisher_object in self.publishers_array:
            self._check_pub_connection(publisher_object)
        rospy.logdebug("ALL SENSORS READY")

    def _check_pub_connection(self, publisher_object):

        rate = rospy.Rate(10)  # 10hz
        while publisher_object.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to publisher_object yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("publisher_object Publisher Connected")

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
    def move_joints(self, joints_array, epsilon=0.05, update_rate=10, time_sleep=0.05, check_position=True):
        """
        It will move the Hopper Joints to the given Joint_Array values
        """
        i = 0
        for publisher_object in self.publishers_array:
            joint_value = Float64()
            joint_value.data = joints_array[i]
            rospy.logdebug("JointsPos>>"+str(joint_value))
            publisher_object.publish(joint_value)
            i += 1

        if check_position:
            self.wait_time_for_execute_movement(
                joints_array, epsilon, update_rate)
        else:
            self.wait_time_movement_hard(time_sleep=time_sleep)

    def wait_time_for_execute_movement(self, joints_array, epsilon, update_rate):
        """
        We wait until Joints are where we asked them to be based on the joints_states
        :param joints_array:Joints Values in radians of each of the three joints of hopper leg.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the joint_states.
        :return:
        """
        rospy.logdebug("START wait_until_twist_achieved...")

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0

        rospy.logdebug("Desired JointsState>>" + str(joints_array))
        rospy.logdebug("epsilon>>" + str(epsilon))

        while not rospy.is_shutdown():
            current_joint_states = self._check_joint_states_ready()

            values_to_check = [current_joint_states.position[0],
                               current_joint_states.position[1],
                               current_joint_states.position[2]]

            vel_values_are_close = self.check_array_similar(
                joints_array, values_to_check, epsilon)

            if vel_values_are_close:
                rospy.logdebug("Reached JointStates!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time)+"]")

        rospy.logdebug("END wait_until_jointstate_achieved...")

        return delta_time

    def wait_time_movement_hard(self, time_sleep):
        """
        Hard Wait to avoid inconsistencies in times executing actions
        """
        rospy.logdebug("Test Wait="+str(time_sleep))
        time.sleep(time_sleep)

    def check_array_similar(self, ref_value_array, check_value_array, epsilon):
        """
        It checks if the check_value id similar to the ref_value
        """
        rospy.logdebug("ref_value_array="+str(ref_value_array))
        rospy.logdebug("check_value_array="+str(check_value_array))
        return numpy.allclose(ref_value_array, check_value_array, atol=epsilon)

    def get_odom(self):
        return self.odom

    def get_imu(self):
        return self.imu

    def get_lowerleg_contactsensor_state(self):
        return self.lowerleg_contactsensor_state

    def get_joint_states(self):
        return self.joint_states
