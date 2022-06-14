import numpy
import rospy
import time
import tf
from openai_ros import robot_gazebo_env
import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import JointLimits
from sensor_msgs.msg import Image
from openai_ros.openai_ros_common import ROSLauncher


class SawyerEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all SawyerEnv environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new SawyerEnv environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /robot/joint_limits: Odometry of the Base of Wamv

        Actuators Topic List:
        * As actuator we will use a class to interface with the movements through commands.

        Args:
        """
        rospy.logdebug("Start SawyerEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="sawyer_gazebo",
                    launch_file_name="put_sawyer_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(SawyerEnv, self).__init__(controllers_list=self.controllers_list,
                                        robot_name_space=self.robot_name_space,
                                        reset_controls=False,
                                        start_init_physics_parameters=False,
                                        reset_world_or_sim="WORLD")

        rospy.logdebug("SawyerEnv unpause...")
        self.gazebo.unpauseSim()
        # self.controllers_object.reset_controllers()

        # TODO: Fill it with the sensors
        self._check_all_systems_ready()

        rospy.Subscriber("/io/internal_camera/head_camera/image_raw",
                         Image, self._head_camera_image_raw_callback)
        rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw",
                         Image, self._right_hand_camera_image_raw_callback)

        self._setup_tf_listener()
        self._setup_movement_system()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished SawyerEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("SawyerEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END SawyerEnv _check_all_systems_ready...")
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        # TODO: Here go the sensors like cameras and joint states
        self._check_head_camera_image_raw_ready()
        self._check_right_hand_camera_image_raw_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_head_camera_image_raw_ready(self):
        self.head_camera_image_raw = None
        rospy.logdebug(
            "Waiting for /io/internal_camera/head_camera/image_raw to be READY...")
        while self.head_camera_image_raw is None and not rospy.is_shutdown():
            try:
                self.head_camera_image_raw = rospy.wait_for_message(
                    "/io/internal_camera/head_camera/image_raw", Image, timeout=5.0)
                rospy.logdebug(
                    "Current /io/internal_camera/head_camera/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /io/internal_camera/head_camera/image_raw not ready yet, retrying for getting head_camera_image_raw")
        return self.head_camera_image_raw

    def _check_right_hand_camera_image_raw_ready(self):
        self.right_hand_camera_image_raw = None
        rospy.logdebug(
            "Waiting for /io/internal_camera/right_hand_camera/image_raw to be READY...")
        while self.right_hand_camera_image_raw is None and not rospy.is_shutdown():
            try:
                self.right_hand_camera_image_raw = rospy.wait_for_message(
                    "/io/internal_camera/right_hand_camera/image_raw", Image, timeout=5.0)
                rospy.logdebug(
                    "Current /io/internal_camera/right_hand_camera/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /io/internal_camera/right_hand_camera/image_raw not ready yet, retrying for getting right_hand_camera_image_raw")
        return self.right_hand_camera_image_raw

    def _head_camera_image_raw_callback(self, data):
        self.head_camera_image_raw = data

    def _right_hand_camera_image_raw_callback(self, data):
        self.right_hand_camera_image_raw = data

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
        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()
        if not valid_limbs:
            rp.log_message(("Cannot detect any limb parameters on this robot. "
                            "Exiting."), "ERROR")
            return

        rospy.loginfo("Valid Sawyer Limbs==>"+str(valid_limbs))

        print("Getting robot state... ")
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled

        rospy.loginfo("Enabling robot...")
        rs.enable()
        self._map_actions_to_movement()

    def _map_actions_to_movement(self, side="right", joint_delta=0.1):

        self.limb = intera_interface.Limb(side)

        try:
            self.gripper = intera_interface.Gripper(side + '_gripper')
        except:
            self.has_gripper = False
            rospy.loginfo("The electric gripper is not detected on the robot.")
        else:
            self.has_gripper = True

        self.joints = self.limb.joint_names()

        self.bindings = {
            self.joints[0]+"_increase": (self.set_j, [self.joints[0], joint_delta], self.joints[0]+" increase"),
            self.joints[0]+"_decrease": (self.set_j, [self.joints[0], -joint_delta], self.joints[0]+" decrease"),
            self.joints[1]+"_increase": (self.set_j, [self.joints[1], joint_delta], self.joints[1]+" increase"),
            self.joints[1]+"_decrease": (self.set_j, [self.joints[1], -joint_delta], self.joints[1]+" decrease"),
            self.joints[2]+"_increase": (self.set_j, [self.joints[2], joint_delta], self.joints[2]+" increase"),
            self.joints[2]+"_decrease": (self.set_j, [self.joints[2], -joint_delta], self.joints[2]+" decrease"),
            self.joints[3]+"_increase": (self.set_j, [self.joints[3], joint_delta], self.joints[3]+" increase"),
            self.joints[3]+"_decrease": (self.set_j, [self.joints[3], -joint_delta], self.joints[3]+" decrease"),
            self.joints[4]+"_increase": (self.set_j, [self.joints[4], joint_delta], self.joints[4]+" increase"),
            self.joints[4]+"_decrease": (self.set_j, [self.joints[4], -joint_delta], self.joints[4]+" decrease"),
            self.joints[5]+"_increase": (self.set_j, [self.joints[5], joint_delta], self.joints[5]+" increase"),
            self.joints[5]+"_decrease": (self.set_j, [self.joints[5], -joint_delta], self.joints[5]+" decrease"),
            self.joints[6]+"_increase": (self.set_j, [self.joints[6], joint_delta], self.joints[6]+" increase"),
            self.joints[6]+"_decrease": (self.set_j, [self.joints[6], -joint_delta], self.joints[6]+" decrease")
        }
        if self.has_gripper:
            self.bindings.update({
                "close": (self.set_g, "close", side+" gripper close"),
                "open": (self.set_g, "open", side+" gripper open"),
                "calibrate": (self.set_g, "calibrate", side+" gripper calibrate")
            })

        rospy.loginfo("Controlling joints...")

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
    def execute_movement(self, action_id):
        """
        It executed the command given through an id. This will move any joint
        of Sawyer, including the gripper if it has it.
        :param: action_id: These are the possible action_id values and the action asociated.

        self.joints[0]+"_increase",
        self.joints[0]+_decrease,
        self.joints[1]+"_increase",
        self.joints[1]+"_decrease",
        self.joints[2]+"_increase",
        self.joints[2]+"_decrease",
        self.joints[3]+"_increase",
        self.joints[3]+"_decrease",
        self.joints[4]+"_increase",
        self.joints[4]+"_decrease",
        self.joints[5]+"_increase",
        self.joints[5]+"_decrease",
        self.joints[6]+"_increase",
        self.joints[6]+"_decrease",
        gripper_close,
        gripper_open,
        gripper_calibrate
        """

        if action_id in self.bindings:
            cmd = self.bindings[action_id]
            if action_id == "gripper_close" or action_id == "gripper_open" or action_id == "gripper_calibrate":
                cmd[0](cmd[1])
                rospy.loginfo("command: %s" % (cmd[2],))
            else:
                # expand binding to something like "self.set_j(right, 'j0', joint_delta)"
                cmd[0](*cmd[1])
                rospy.loginfo("command: %s" % (cmd[2],))
        else:
            rospy.logerr("NOT VALID key binding, it should be one of these: ")
            for key, val in sorted(self.bindings.items(),
                                   key=lambda x: x[1][2]):
                rospy.logerr("  %s: %s" % (key, val[2]))

    def set_j(self, joint_name, delta):
        current_position = self.limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        self.limb.set_joint_positions(joint_command)

    def set_g(self, action):
        if self.has_gripper:
            if action == "close":
                self.gripper.close()
            elif action == "open":
                self.gripper.open()
            elif action == "calibrate":
                self.gripper.calibrate()

    def move_joints_to_angle_blocking(self, joint_positions_dict, timeout=15.0, threshold=0.008726646):
        """
        It moves all the joints to the given position and doesnt exit until it reaches that position
        """
        self.limb.move_to_joint_positions(positions=joint_positions_dict,
                                          timeout=15.0,
                                          threshold=0.008726646,
                                          test=None)

    def get_limb_joint_names_array(self):
        """
        Returns the Joint Names array of the Limb.
        """
        return self.joints

    def get_all_limb_joint_angles(self):
        """
        Return dictionary dict({str:float}) with all the joints angles
        """
        return self.limb.joint_angles()

    def get_all_limb_joint_efforts(self):
        """
        Returns a dictionary dict({str:float}) with all the joints efforts
        """
        return self.limb.joint_efforts()

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
        while (trans is None or rot is None) and not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform(
                    start_frame, end_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("TF start to end not ready YET...")
                duration_obj = rospy.Duration.from_sec(1.0)
                rospy.sleep(duration_obj)

        return trans, rot

    def check_joint_limits_ready(self):
        self.joint_limits = None
        rospy.logdebug("Waiting for /robot/joint_limits to be READY...")
        while self.joint_limits is None and not rospy.is_shutdown():
            try:
                self.joint_limits = rospy.wait_for_message(
                    "/robot/joint_limits", JointLimits, timeout=3.0)
                rospy.logdebug("Current /robot/joint_limits READY=>")

            except:
                rospy.logerr(
                    "Current /robot/joint_limits not ready yet, retrying for getting joint_limits")
        return self.joint_limits

    def get_joint_limits(self):
        return self.joint_limits

    def get_head_camera_image_raw(self):
        return self.head_camera_image_raw

    def get_right_hand_camera_image_raw(self):
        return self.right_hand_camera_image_raw

    def init_joint_limits(self):
        """
        Get the Joint Limits, in the init fase where we need to unpause the simulation to get them
        :return: joint_limits: The Joint Limits Dictionary, with names, angles, vel and effort limits.
        """
        self.gazebo.unpauseSim()
        joint_limits = self.check_joint_limits_ready()
        self.gazebo.pauseSim()
        return joint_limits
