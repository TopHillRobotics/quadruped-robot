import numpy as np
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from openai_ros import robot_gazebo_env
from openai_ros.openai_ros_common import ROSLauncher


class FetchSimpleEnv(robot_gazebo_env.RobotGazeboEnv):

    def __init__(self, ros_ws_abspath):
        rospy.logdebug("Entered Fetch Env")

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="fetch_simple_description",
                    launch_file_name="put_fetchsimple_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        self.controllers_list = ["joint_state_controller",
                              "torso_lift_joint_position_controller",
                              "bellows_joint_position_controller",
                              "head_pan_joint_position_controller",
                              "head_tilt_joint_position_controller",
                              "shoulder_pan_joint_position_controller",
                              "shoulder_lift_joint_position_controller",
                              "upperarm_roll_joint_position_controller",
                              "elbow_flex_joint_position_controller",
                              "forearm_roll_joint_position_controller",
                              "wrist_flex_joint_position_controller",
                              "wrist_roll_joint_position_controller",
                              "r_gripper_finger_joint_position_controller",
                              "l_gripper_finger_joint_position_controller"]

        self.robot_name_space = "fetch"
        self.reset_controls = True

        super(FetchSimpleEnv, self).__init__(controllers_list=self.controllers_list,
                                             robot_name_space=self.robot_name_space,
                                             reset_controls=self.reset_controls,
                                             start_init_physics_parameters=False,
                                             reset_world_or_sim="WORLD")

        # We Start all the ROS related Subscribers and publishers

        self.JOINT_STATES_SUBSCRIBER = '/fetch/joint_states'
        self.join_names = ["joint0",
                           "joint1",
                           "joint2",
                           "joint3",
                           "joint4",
                           "joint5",
                           "joint6"]

        self.gazebo.unpauseSim()
        # Start Move Fetch Object, that checks all systems are ready
        self.move_fetch_object = FetchSimpleMove()
        # Wait until Fetch goes to the init pose
        self.move_fetch_object.init_position()

        # We pause until the next step
        self.gazebo.pauseSim()

    # RobotGazeboEnv virtual methods
    # ----------------------------

    def move_to_init_pose(self):
        self.move_fetch_object.init_position()

    def get_joint_limits(self):
        return self.move_fetch_object.joint_upper_limits, self.move_fetch_object.joint_lower_limits

    def get_joints_position(self):
        return self.move_fetch_object.get_current_joints_position()

    def set_trajectory_joints(self, delta_joints_array):
        self.move_fetch_object.delta_joints(delta_joints_array)
        return True

    # ParticularEnv methods
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self.move_fetch_object.check_all_systems_ready()
        return True


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


class FetchSimpleMove(object):
    def __init__(self):
        rospy.loginfo("Initialising...")

        self.name_joints = ["bellows_joint",
                            "elbow_flex_joint",
                            "forearm_roll_joint",
                            "head_pan_joint",
                            "head_tilt_joint",
                            "l_gripper_finger_joint",
                            "r_gripper_finger_joint",
                            "shoulder_lift_joint",
                            "shoulder_pan_joint",
                            "torso_lift_joint",
                            "upperarm_roll_joint",
                            "wrist_flex_joint",
                            "wrist_roll_joint"]

        self.joint_upper_limits = [0.4,
                                   2.251,
                                   6.27,  # Was none but placed a limit
                                   1.57,
                                   1.45,
                                   0.05,
                                   0.05,
                                   1.518,
                                   1.6056,
                                   0.38615,
                                   6.27,  # Was none but placed a limit
                                   2.16,
                                   6.27,  # Was none but placed a limit
                                   ]
        self.joint_lower_limits = [0.0,
                                   -2.251,
                                   0.0,  # Was none but placed a limit
                                   -1.57,
                                   -0.76,
                                   0.0,
                                   0.0,
                                   -1.221,
                                   -1.6056,
                                   0.0,
                                   0.0,  # Was none but placed a limit
                                   -2.16,
                                   0.0,  # Was none but placed a limit
                                   ]

        self.travel_arm_pose = [0.0,
                                -1.8,  # elbow_flex_joint
                                0.0,  # forearm_roll_joint
                                0.0,
                                0.0,
                                0.05,
                                0.04,
                                0.7,  # shoulder_lift_joint
                                1.32,  # shoulder_pan_joint
                                0.0,  # upperarm_roll_joint
                                0.0,
                                -0.4,  # wrist_flex_joint
                                0.1]

        self.joint_array = len(self.name_joints)*[0.0]

        self.pub_position_array = []
        for joint in self.name_joints:
            topic_name = "/fetch/"+joint+"_position_controller/command"
            self.pub_position_array.append(
                rospy.Publisher(topic_name, Float64, queue_size=1))

        # Wait for publishers to be ready
        self.check_all_systems_ready()

        rospy.Subscriber("/fetch/joint_states", JointState,
                         self.join_state_callback)

    def check_all_systems_ready(self):
        self.wait_publishers_to_be_ready()
        self._check_joint_states_ready()

    def _check_joint_states_ready(self):
        self.joints_state = None
        while self.joints_state is None and not rospy.is_shutdown():
            try:
                self.joints_state = rospy.wait_for_message(
                    "/fetch/joint_states", JointState, timeout=1.0)
                rospy.logdebug(
                    "Current /fetch/joint_states READY=>" + str(self.joints_state))

            except:
                rospy.logerr(
                    "Current /fetch/joint_states not ready yet, retrying for getting joint_states")
        return self.joints_state

    def join_state_callback(self, msg):
        self.joints_state = msg

    def get_current_joints_position(self):
        return self.joints_state.position

    def init_position(self):
        # We wait what it takes to reset pose
        self.move_all_joints(joints_pos_array=self.joint_array, time_out=0.0)

    def set_travel_arm_pose(self):
        self.move_all_joints(joints_pos_array=self.travel_arm_pose)

    def wait_for_joints_to_get_there(self, desired_pos_array, error=0.2, timeout=3.0):

        time_waiting = 0.0
        frequency = 10.0
        are_equal = False
        is_timeout = False
        rate = rospy.Rate(frequency)
        rospy.logwarn("Waiting for joint to get to the position")
        while not are_equal and not is_timeout and not rospy.is_shutdown():

            current_pos = [self.joints_state.position]

            are_equal = np.allclose(a=current_pos,
                                    b=desired_pos_array,
                                    atol=error)

            rospy.logdebug("are_equal="+str(are_equal))
            rospy.logdebug(str(desired_pos_array))
            rospy.logdebug(str(current_pos))

            rate.sleep()
            if timeout == 0.0:
                # We wait what it takes
                time_waiting += 0.0
            else:
                time_waiting += 1.0 / frequency
            is_timeout = time_waiting > timeout

        rospy.logwarn(
            "Joints are in the desired position with an erro of "+str(error))

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        rospy.logdebug("Waiting for Publishers to be ready...")
        i = 0
        for publisher_obj in self.pub_position_array:
            publisher_ready = False
            while not publisher_ready:
                connection_num = publisher_obj.get_num_connections()
                publisher_ready = connection_num > 0
                rospy.logdebug("Pub joint NOT Ready=" +
                               str(self.name_joints[i]))
                rate_wait.sleep()
            rospy.logdebug("Publisher for joint Ready=" +
                           str(self.name_joints[i]))
            i += 1

    def move_all_joints(self, joints_pos_array, time_out=3.0, error=0.2):

        assert len(joints_pos_array) == len(
            self.joint_array), "Lengths dont match"
        i = 0
        for angle in joints_pos_array:
            angle_msg = Float64()
            angle_msg.data = angle
            # Publish Joint Position
            self.pub_position_array[i].publish(angle_msg)
            i += 1

        self.wait_for_joints_to_get_there(self.joint_array, error=error, timeout=time_out)
        self.update_joints(new_joints_pos=joints_pos_array)

    def update_joints(self, new_joints_pos):

        i = 0

        assert len(new_joints_pos) == len(
            self.joint_array), "Lengths don't match in Update"

        for new_joint_value in new_joints_pos:
            upper = self.joint_upper_limits[i]
            lower = self.joint_lower_limits[i]
            if upper is None or lower is None:
                self.joint_array[i] = new_joint_value
            else:
                if upper >= new_joint_value >= lower:
                    self.joint_array[i] = new_joint_value
                elif new_joint_value < lower:
                    self.joint_array[i] = lower
                else:
                    self.joint_array[i] = upper
            rospy.logdebug("index =" + str(i))
            rospy.logdebug("length of name_joints =" +
                           str(len(self.name_joints[i])))
            rospy.logdebug("name_joints=" + str(self.name_joints[i]))

            i += 1

    def delta_joints(self, delta_array):
        """
        delta_array = [bellows_joint, elbow_flex_joint, forearm_roll_joint, head_pan_joint, head_tilt_joint,
  l_gripper_finger_joint, r_gripper_finger_joint, shoulder_lift_joint, shoulder_pan_joint,
  torso_lift_joint, upperarm_roll_joint, wrist_flex_joint, wrist_roll_joint]
        :param delta_array:
        :return:
        """
        new_pos_array = len(delta_array)*[0.0]
        i = 0
        for delta in delta_array:
            new_pos_array[i] = self.joint_array[i] + delta
            i += 1

        self.move_all_joints(new_pos_array)

    def get_current_angles(self):
        return self.joint_array
