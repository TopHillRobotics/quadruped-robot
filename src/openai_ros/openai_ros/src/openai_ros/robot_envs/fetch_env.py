import numpy as np
import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from sensor_msgs.msg import JointState
from openai_ros import robot_gazebo_env
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from openai_ros.openai_ros_common import ROSLauncher


class FetchEnv(robot_gazebo_env.RobotGazeboEnv):

    def __init__(self, ros_ws_abspath):
        rospy.logdebug("========= In Fetch Env")

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="fetch_gazebo",
                    launch_file_name="put_robot_in_world_HER.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # this object contains all object's positions!!
        self.obj_positions = Obj_Pos()

        self.controllers_list = []

        self.robot_name_space = ""
        self.reset_controls = False

        super(FetchEnv, self).__init__(controllers_list=self.controllers_list,
                                       robot_name_space=self.robot_name_space,
                                       reset_controls=False,
                                       start_init_physics_parameters=False,
                                       reset_world_or_sim="WORLD")

        # We Start all the ROS related Subscribers and publishers

        self.JOINT_STATES_SUBSCRIBER = '/joint_states'
        self.join_names = ["joint0",
                           "joint1",
                           "joint2",
                           "joint3",
                           "joint4",
                           "joint5",
                           "joint6"]

        self.gazebo.unpauseSim()
        self._check_all_systems_ready()

        self.joint_states_sub = rospy.Subscriber(
            self.JOINT_STATES_SUBSCRIBER, JointState, self.joints_callback)
        self.joints = JointState()

        # Start Services
        self.move_fetch_object = MoveFetch()

        # Wait until it has reached its Sturtup Position
        self.wait_fetch_ready()

        self.gazebo.pauseSim()
        # Variables that we give through the constructor.

        rospy.logdebug("========= Out Fetch Env")

    # RobotGazeboEnv virtual methods
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # FetchEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()

        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message(
                    self.JOINT_STATES_SUBSCRIBER, JointState, timeout=1.0)
                rospy.logdebug(
                    "Current "+str(self.JOINT_STATES_SUBSCRIBER)+" READY=>" + str(self.joints))

            except:
                rospy.logerr(
                    "Current "+str(self.JOINT_STATES_SUBSCRIBER)+" not ready yet, retrying....")
        return self.joints

    def joints_callback(self, data):
        self.joints = data

    def get_joints(self):
        return self.joints

    def get_joint_names(self):
        return self.joints.name

    def set_trajectory_ee(self, action):
        """
        Sets the Pose of the EndEffector based on the action variable.
        The action variable contains the position and orientation of the EndEffector.
        See create_action
        """
        # Set up a trajectory message to publish.
        ee_target = geometry_msgs.msg.Pose()

        ee_target.orientation.x = -0.707
        ee_target.orientation.y = 0.0
        ee_target.orientation.z = 0.707
        ee_target.orientation.w = 0.001

        ee_target.position.x = action[0]
        ee_target.position.y = action[1]
        ee_target.position.z = action[2]

        result = self.move_fetch_object.ee_traj(ee_target)
        return result

    def set_trajectory_joints(self, initial_qpos):

        positions_array = [None] * 7
        positions_array[0] = initial_qpos["joint0"]
        positions_array[1] = initial_qpos["joint1"]
        positions_array[2] = initial_qpos["joint2"]
        positions_array[3] = initial_qpos["joint3"]
        positions_array[4] = initial_qpos["joint4"]
        positions_array[5] = initial_qpos["joint5"]
        positions_array[6] = initial_qpos["joint6"]

        self.move_fetch_object.joint_traj(positions_array)

        return True

    def create_action(self, position, orientation):
        """
        position = [x,y,z]
        orientation= [x,y,z,w]
        """

        gripper_target = np.array(position)
        gripper_rotation = np.array(orientation)
        action = np.concatenate([gripper_target, gripper_rotation])

        return action

    def create_joints_dict(self, joints_positions):
        """
        Based on the Order of the positions, they will be assigned to its joint name
        names_in_order:
          joint0: 0.0
          joint1: 0.0
          joint2: 0.0
          joint3: -1.5
          joint4: 0.0
          joint5: 1.5
          joint6: 0.0
        """

        assert len(joints_positions) == len(
            self.join_names), "Wrong number of joints, there should be "+str(len(self.join_names))
        joints_dict = dict(zip(self.join_names, joints_positions))

        return joints_dict

    def get_ee_pose(self):
        """
        Returns geometry_msgs/PoseStamped
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        """
        self.gazebo.unpauseSim()
        gripper_pose = self.move_fetch_object.ee_pose()
        self.gazebo.pauseSim()
        return gripper_pose

    def get_ee_rpy(self):
        gripper_rpy = self.move_fetch_object.ee_rpy()
        return gripper_rpy

    def wait_fetch_ready(self):
        """
        # TODO: Make it wait for this position
        Desired Position to wait for

        (0.44291739197591884,
        -0.13691381375054146,
        -4.498589757905556e-09,
        0.006635104153645881,
        0.0018354466563206273,
        0.0023142971818792546,
        1.3200059164171716,
        1.399964660857453,
        -0.19981518020955402,
        1.719961735970255,
        1.0394665737933906e-05,
        1.659980987917125,
        -6.067103113238659e-06,
        0.05001918351472232,
        0.050051597253287436)
        """
        import time
        for i in range(20):
            print("WAITING..."+str(i))
            sys.stdout.flush()
            time.sleep(1.0)

        print("WAITING...DONE")

    # ParticularEnv methods
    # ----------------------------

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


class Obj_Pos(object):
    """
    This object maintains the pose and rotation of the cube in a simulation through Gazebo Service

    """

    def __init__(self):
        world_specs = rospy.ServiceProxy(
            '/gazebo/get_world_properties', GetWorldProperties)()
        self.time = 0
        self.model_names = world_specs.model_names
        self.get_model_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

    def get_states(self):
        """
        Returns the ndarray of pose&rotation of the cube
        """
        for model_name in self.model_names:
            if model_name == "cube":
                data = self.get_model_state(
                    model_name, "world")  # gazebo service client
                return np.array([
                    data.pose.position.x,
                    data.pose.position.y,
                    data.pose.position.z,
                    data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z
                ])


class MoveFetch(object):
    def __init__(self):
        rospy.logdebug("===== In MoveFetch")
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("===== Out MoveFetch")

    def ee_traj(self, pose):
        self.group.set_pose_target(pose)
        result = self.execute_trajectory()
        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory()

        return result

    def execute_trajectory(self):
        """
        Assuming that the trajecties has been set to the self objects appropriately
        Make a plan to the destination in Homogeneous Space(x,y,z,yaw,pitch,roll)
        and returns the result of execution
        """
        self.plan = self.group.plan()
        result = self.group.go(wait=True)
        return result

    def ee_pose(self):
        gripper_pose = self.group.get_current_pose()
        return gripper_pose

    def ee_rpy(self, request):
        gripper_rpy = self.group.get_current_rpy()
        return gripper_rpy
