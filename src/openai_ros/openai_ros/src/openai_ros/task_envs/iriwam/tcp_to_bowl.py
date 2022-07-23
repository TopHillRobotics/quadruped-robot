import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import iriwam_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class IriWamTcpToBowlEnv(iriwam_env.IriWamEnv):
    def __init__(self):
        """
        Make iriwam learn how pick up a cube
        """
        # We set an initial value imposible to ach to reward at least the first time
        self.previous_distance_from_block = 100.0

        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/iriwam/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="iri_wam_gazebo",
                    launch_file_name="start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/iriwam/config",
                               yaml_file_name="tcp_to_bowl.yaml")

        # We execute this one before because there are some functions that this
        # TaskEnv uses that use variables from the parent class, like the effort limit fetch.
        super(IriWamTcpToBowlEnv, self).__init__(ros_ws_abspath)

        # Here we will add any init functions prior to starting the MyRobotEnv

        # Only variable needed to be set here

        rospy.logdebug("Start IriWamTcpToBowlEnv INIT...")
        number_actions = rospy.get_param('/iriwam/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        self.iri_wam_joint_1 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_1")
        self.iri_wam_joint_2 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_2")
        self.iri_wam_joint_3 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_3")
        self.iri_wam_joint_4 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_4")
        self.iri_wam_joint_5 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_5")
        self.iri_wam_joint_6 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_6")
        self.iri_wam_joint_7 = rospy.get_param(
            "/iriwam/init_joints/iri_wam_joint_7")

        self.init_joints_positions_array = [self.iri_wam_joint_1,
                                            self.iri_wam_joint_2,
                                            self.iri_wam_joint_3,
                                            self.iri_wam_joint_4,
                                            self.iri_wam_joint_5,
                                            self.iri_wam_joint_6,
                                            self.iri_wam_joint_7]

        self.init_joints_x_max = rospy.get_param(
            "/iriwam/work_space/x_max")
        self.init_joints_y_max = rospy.get_param(
            "/iriwam/work_space/y_max")
        self.init_joints_z_max = rospy.get_param(
            "/iriwam/work_space/z_max")
        self.init_joints_x_min = rospy.get_param(
            "/iriwam/work_space/x_min")
        self.init_joints_y_min = rospy.get_param(
            "/iriwam/work_space/y_min")
        self.init_joints_z_min = rospy.get_param(
            "/iriwam/work_space/z_min")

        self.joint_increment_value = rospy.get_param(
            "/iriwam/joint_increment_value")

        self.max_distance_from_red_bowl = rospy.get_param(
            "/iriwam/max_distance_from_red_bowl")
        self.min_distance_from_red_bowl = rospy.get_param(
            "/iriwam/min_distance_from_red_bowl")

        self.min_laser_distance = rospy.get_param("/iriwam/min_laser_distance")

        self.dec_obs = rospy.get_param("/iriwam/number_decimals_precision_obs")

        # We place the Maximum and minimum values of observations
        # TODO: Fill when get_observations is done.
        """
        We supose that its all these:
        head_pan, right_gripper_l_finger_joint, right_gripper_r_finger_joint, right_j0, right_j1,
  right_j2, right_j3, right_j4, right_j5, right_j6

        Plus the first three are the block_to_tcp vector
        """

        # We fetch the limits of the joinst to get the effort and angle limits
        self.joint_limits = self.init_joint_limits()

        high = numpy.array([self.init_joints_x_max,
                            self.init_joints_y_max,
                            self.init_joints_z_max,
                            self.joint_limits[0].max_position,
                            self.joint_limits[1].max_position,
                            self.joint_limits[2].max_position,
                            self.joint_limits[3].max_position,
                            self.joint_limits[4].max_position,
                            self.joint_limits[5].max_position,
                            self.joint_limits[6].max_position
                            ])

        low = numpy.array([self.init_joints_x_min,
                           self.init_joints_y_min,
                           self.init_joints_z_min,
                           self.joint_limits[0].min_position,
                           self.joint_limits[1].min_position,
                           self.joint_limits[2].min_position,
                           self.joint_limits[3].min_position,
                           self.joint_limits[4].min_position,
                           self.joint_limits[5].min_position,
                           self.joint_limits[6].min_position
                           ])

        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))

        # Rewards

        self.done_reward = rospy.get_param("/iriwam/done_reward")
        self.closer_to_block_reward = rospy.get_param(
            "/iriwam/closer_to_block_reward")

        self.cumulated_steps = 0.0

        # We init the CVBridge object
        self.bridge_object = CvBridge()

        rospy.logdebug("END IriWamTcpToBowlEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the two proppelers speed to 0.0 and waits for the time_sleep
        to allow the action to be executed
        """

        # We set the angles of the IriWam to the init pose:
        self.move_joints_to_angle_blocking(self.init_joints_positions_array)

        self.joints = []
        for joint_value in self.init_joints_positions_array:
            self.joints.append(joint_value)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """

        # For Info Purposes
        self.cumulated_reward = 0.0

        image_data = self.get_camera_rgb_image_raw()
        self.previous_distance_from_bowl = self.get_magnitud_tcp_to_block(
            data=image_data)

    def _set_action(self, action):
        """
        It sets the joints of iriwam based on the action integer given
        based on the action number given.
        :param action: The action integer that sets what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))

        if action == 0:  # Increase joint_0
            self.joints[0] += self.joint_increment_value
        elif action == 1:  # Decrease joint_0
            self.joints[0] -= self.joint_increment_value
        elif action == 2:  # Increase joint_1
            self.joints[1] += self.joint_increment_value
        elif action == 3:  # Decrease joint_1
            self.joints[1] -= self.joint_increment_value
        elif action == 4:  # Increase joint_2
            self.joints[2] += self.joint_increment_value
        elif action == 5:  # Decrease joint_2
            self.joints[2] -= self.joint_increment_value
        elif action == 6:  # Increase joint_3
            self.joints[3] += self.joint_increment_value
        elif action == 7:  # Decrease joint_3
            self.joints[3] -= self.joint_increment_value
        elif action == 8:  # Increase joint_4
            self.joints[4] += self.joint_increment_value
        elif action == 9:  # Decrease joint_4
            self.joints[4] -= self.joint_increment_value
        elif action == 10:  # Increase joint_5
            self.joints[5] += self.joint_increment_value
        elif action == 11:  # Decrease joint_5
            self.joints[5] -= self.joint_increment_value
        elif action == 12:  # Increase joint_6
            self.joints[6] += self.joint_increment_value
        elif action == 13:  # Decrease joint_6
            self.joints[6] -= self.joint_increment_value
        else:
            rospy.logdebug("Action not supported="+str(action))

        # We tell iriwam the action to perform
        self.move_joints_to_angle_blocking(self.joints)

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        iriwamEnv API DOCS.
        :return: observation
        """
        rospy.logdebug("Start Get Observation ==>")

        # We get Join state currently of all the joints
        join_state = self.get_joint_state()
        joints_angles_array = join_state.actual.positions
        joints_angles_array_round = numpy.around(
            joints_angles_array, decimals=self.dec_obs)

        # We get the Laser reading of the center laser ray, only one
        laser_data = self.get_laser_scan()
        center_laser_distance = laser_data.ranges[int(
            len(laser_data.ranges)/2)]
        center_laser_distance_array = [
            round(center_laser_distance, self.dec_obs)]

        # We get the distance laser tip to the red bowl using the image blob detection system
        image_data = self.get_camera_rgb_image_raw()
        distance_from_bowl = self.get_magnitud_tcp_to_block(data=image_data)
        distance_from_bowl_array = [round(distance_from_bowl, self.dec_obs)]

        # We concatenate the two rounded arrays and convert them to standard Python list
        observation = numpy.concatenate(
            (joints_angles_array_round, center_laser_distance_array, distance_from_bowl_array), axis=0).tolist()

        return observation

    def _is_done(self, observations):
        """
        We consider the episode done if:
        1) The iriwam end effector to bowl distance exceeds the maximum
        2) The iriwam end effector to bowl distance reaches the minimum and laser distance is lower than minimum

        """

        distance_laser = observations[-2]
        magnitude_image = observations[-1]

        has_reached_the_block = self.reached_bowl(
            distance_laser, magnitude_image)
        too_faraway_bowl_b = self.too_faraway_bowl(distance_laser, magnitude_image)

        done = has_reached_the_block or too_faraway_bowl_b

        rospy.logdebug("#### IS DONE ? ####")
        rospy.logwarn("done ?="+str(done)+",has_reached_the_block="+str(has_reached_the_block)+",too_faraway_bowl_b="+str(too_faraway_bowl_b))
        rospy.logdebug("#### #### ####")

        return done

    def _compute_reward(self, observations, done):
        """
        We Base the rewards in if its done or not and we base it on
        if the distance to the block has increased or not.
        :return:
        """

        tf_tcp_to_block_vector = Vector3()
        tf_tcp_to_block_vector.x = observations[0]
        tf_tcp_to_block_vector.y = observations[1]
        tf_tcp_to_block_vector.z = observations[2]

        distance_block_to_tcp = self.get_magnitud_tf_tcp_to_block(
            tf_tcp_to_block_vector)
        distance_difference = distance_block_to_tcp - self.previous_distance_from_block

        if not done:

            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logdebug("DECREASE IN DISTANCE GOOD")
                reward = self.closer_to_block_reward
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                #reward = -1*self.closer_to_block_reward
                reward = 0.0

        else:
            distance_laser = observations[-2]
            magnitude_image = observations[-1]

            reached_the_bowl = self.reached_bowl(distance_laser, magnitude_image)
            if reached_the_bowl:
                reward = self.done_reward
            else:
                reward = -1*self.done_reward

        self.previous_distance_from_block = distance_block_to_tcp

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward

    # Internal TaskEnv Methods

    def is_arm_stuck(self, joints_efforts_dict):
        """
        Checks if the efforts in the arm joints exceed certain theshhold
        We will only check the joints_0,1,2,3,4,5,6
        """
        is_arm_stuck = False

        for joint_name in self.joint_limits.joint_names:
            if joint_name in joints_efforts_dict:

                effort_value = joints_efforts_dict[joint_name]
                index = self.joint_limits.joint_names.index(joint_name)
                effort_limit = self.joint_limits.effort[index]

                rospy.logdebug("Joint Effort ==>Name="+str(joint_name) +
                               ",Effort="+str(effort_value)+",Limit="+str(effort_limit))

                if abs(effort_value) > effort_limit:
                    is_arm_stuck = True
                    rospy.logerr("Joint Effort TOO MUCH ==>" +
                                 str(joint_name)+","+str(effort_value))
                    break
                else:
                    rospy.logdebug("Joint Effort is ok==>" +
                                   str(joint_name)+","+str(effort_value))
            else:
                rospy.logdebug(
                    "Joint Name is not in the effort dict==>"+str(joint_name))

        return is_arm_stuck

    def reached_bowl(self, distance_laser, magnitude_image):
        """
        It return True if the distance red by the laser is smaller than minimum and
        the distance by image is smaller than minimum
        """

        laser_close_enough = (distance_laser <= self.min_laser_distance)
        magnitude_image_enough = (
            magnitude_image <= self.min_distance_from_red_bowl)
        rospy.logwarn("reached_bowl condition, magnitude_image==>"+str(magnitude_image)+"<=?"+str(self.min_distance_from_red_bowl))

        # reached_block_b = laser_close_enough and magnitude_image_enough
        reached_block_b = magnitude_image_enough

        rospy.logdebug("###### REACHED Bowl ? ######")
        # rospy.logdebug("laser_close_enough==>"+str(laser_close_enough))
        rospy.logdebug("magnitude_image_enough==>"+str(magnitude_image_enough))
        rospy.logdebug("reached_block_b==>"+str(reached_block_b))
        rospy.logdebug("############")

        return reached_block_b

    def too_faraway_bowl(self, distance_laser, magnitude_image):
        """
        It return True if the distance by image is bigger than maximum
        """

        laser_close_enough = (distance_laser <= self.min_laser_distance)
        magnitude_image_too_big = (
            magnitude_image <= self.max_distance_from_red_bowl)

        too_faraway_b = laser_close_enough and magnitude_image_too_big

        rospy.logdebug("###### Too Faar from bowl ? ######")
        rospy.logdebug("magnitude_image_too_big==>" +
                       str(magnitude_image_too_big))
        rospy.logdebug("too_faraway_b==>"+str(too_faraway_b))
        rospy.logdebug("############")

        return too_faraway_b

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

    def get_magnitud_tf_tcp_to_block(self, translation_vector):
        """
        Given a Vector3 Object, get the magnitud
        :param p_end:
        :return:
        """
        a = numpy.array((translation_vector.x,
                         translation_vector.y,
                         translation_vector.z))

        distance = numpy.linalg.norm(a)

        return distance

    def get_orientation_euler(self, quaternion_vector):
        # We convert from quaternions to euler
        orientation_list = [quaternion_vector.x,
                            quaternion_vector.y,
                            quaternion_vector.z,
                            quaternion_vector.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def is_inside_workspace(self, current_position):
        """
        Check if the iriwam is inside the Workspace defined
        """
        is_inside = False

        rospy.logdebug("##### INSIDE WORK SPACE? #######")
        rospy.logdebug("XYZ current_position"+str(current_position))
        rospy.logdebug("init_joints_x_max"+str(self.init_joints_x_max) +
                       ",init_joints_x_min="+str(self.init_joints_x_min))
        rospy.logdebug("init_joints_y_max"+str(self.init_joints_y_max) +
                       ",init_joints_y_min="+str(self.init_joints_y_min))
        rospy.logdebug("init_joints_z_max"+str(self.init_joints_z_max) +
                       ",init_joints_z_min="+str(self.init_joints_z_min))
        rospy.logdebug("############")

        if current_position.x > self.init_joints_x_min and current_position.x <= self.init_joints_x_max:
            if current_position.y > self.init_joints_y_min and current_position.y <= self.init_joints_y_max:
                if current_position.z > self.init_joints_z_min and current_position.z <= self.init_joints_z_max:
                    is_inside = True

        return is_inside

    def get_magnitud_tcp_to_block(self, data):
        """
        Retrieves the distance end effector laser element to the red bowl through the
        image data given

        :param: data: RGB image data
        :return: magnitude: Distance in pixels from the center of the black blob ( the laser)
        To the center of the red blob ( the red bowl)
        Bear in mind that if the laser tip goes out of the cameras view, it will give a false positive
        """

        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            cv_image = None

        if cv_image is not None:
            # We get image dimensions and crop the parts of the image we don't need
            # Bear in mind that because the first value of the image matrix is start and second value is down limit.
            # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
            # To make process faster.
            height, width, channels = cv_image.shape
            descentre = int(-height/2)
            rows_to_watch = int(height)
            aux1 = int((height)/2+descentre)
            aux2 = int((height) / 2+(descentre+rows_to_watch))
            rospy.logdebug("aux1="+str(aux1)+",aux2="+str(aux2)+",width"+str(width))
            crop_img = cv_image[aux1:aux2][1:width]

            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # We track two colours, the RedBowl and the Black tip of IriWam Arm ( laser ) which is black.

            # RED BOWL

            lower_red = numpy.array([0, 204, 100])
            upper_red = numpy.array([0, 255, 255])

            # Threshold the HSV image to get only yellow colors
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask, False)
            try:
                cx_red, cy_red = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy_red, cx_red = height/2, width/2

            # Black Laser
            lower_black = numpy.array([0, 0, 0])
            upper_black = numpy.array([0, 0, 10])

            # Threshold the HSV image to get only yellow colors
            mask_black = cv2.inRange(hsv, lower_black, upper_black)

            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask_black, False)
            try:
                cx_black, cy_black = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy_black, cx_black = height/2, width/2

            # Bitwise-AND mask and original image
            res_black = cv2.bitwise_and(crop_img, crop_img)

            # Draw the centroid in the resultut image
            cv2.circle(res_black, (int(cx_red), int(cy_red)), 10, (255, 0, 0), -1)
            cv2.circle(res_black, (int(cx_black), int(cy_black)),
                       10, (0, 255, 0), -1)

            cv2.imshow("RES BLACK", res_black)

            cv2.waitKey(1)

            error_x = cx_red - cx_black
            error_y = cy_red - cy_black
            error_array = numpy.array([error_x, error_y])
            magnitude = numpy.linalg.norm(error_array)
            rospy.logdebug("Magnitude==>"+str(magnitude))
        else:
            magnitude = 10.0

        return magnitude

    def clean_up(self):
        cv2.destroyAllWindows()
