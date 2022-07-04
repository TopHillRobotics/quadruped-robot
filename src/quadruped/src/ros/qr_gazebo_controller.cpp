// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ros/qr_gazebo_controller.h"

bool startControllers(ros::NodeHandle &nodeHandle, std::string serviceName, std::vector<std::string> &controllersToStart) {

    ros::ServiceClient switchController = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.start_controllers = controllersToStart;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);

    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller start correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to start controller");
        return false;
    }
}

bool stopControllers(ros::NodeHandle &nodeHandle, std::string serviceName, std::vector<std::string> &controllersToStop) {

    ros::ServiceClient switchController = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.stop_controllers = controllersToStop;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    switchControllerMsg.request.start_asap = false;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);

    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller stop correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to stop controller");
        return false;
    }
}

bool ResetRobotBySystem() {

    int deleteModelId = system("rosservice call gazebo/delete_model '{model_name: a1_gazebo}'");
    ROS_INFO("delete state: %d", deleteModelId);

    int urdfStateId = system("rosrun gazebo_ros spawn_model -urdf -z 0.6 -model a1_gazebo -param robot_description -unpause");
    ROS_INFO("spawn model state: %d", urdfStateId);

    int controllersStateId = system("rosrun controller_manager spawner __ns:=/a1_gazebo joint_state_controller "\
          "FL_hip_controller FL_thigh_controller FL_calf_controller "\
          "FR_hip_controller FR_thigh_controller FR_calf_controller "\
          "RL_hip_controller RL_thigh_controller RL_calf_controller "\
          "RR_hip_controller RR_thigh_controller RR_calf_controller &");
    ROS_INFO("controller statu: %d", controllersStateId);

    sleep(1);
    return true;
}

bool ResetRobotByService(ros::ServiceClient &modelStateClient, ros::ServiceClient &jointStateClient) {
    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::SetModelConfiguration setjointstate;

    modelState.model_name = "a1_gazebo";
    modelState.reference_frame = "world";

    geometry_msgs::Twist model_twist;
    geometry_msgs::Pose model_pose;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    model_pose.position.x =0;
    model_pose.position.y =0;
    model_pose.position.z =0.3;
    model_pose.orientation.w = 1;
    model_pose.orientation.x = 0;
    model_pose.orientation.y = 0;
    model_pose.orientation.z = 0;

    modelState.twist = model_twist;
    modelState.pose = model_pose;

    setmodelstate.request.model_state = modelState;
    setjointstate.request.model_name = "a1_gazebo";
    setjointstate.request.urdf_param_name = "robot_description";
    setjointstate.request.joint_names = {"FR_hip_joint","FR_thigh_joint", "FR_calf_joint",
                                         "FL_hip_joint","FL_thigh_joint", "FL_calf_joint",
                                         "RR_hip_joint","RR_thigh_joint", "RR_calf_joint",
                                         "RL_hip_joint","RL_thigh_joint", "RL_calf_joint"};
    double hip_angle = 0.3;
    double thigh_angle = 1.1;
    double calf_angle = -2.2;

    setjointstate.request.joint_positions = {-hip_angle,thigh_angle,calf_angle,
                                             hip_angle,thigh_angle,calf_angle,
                                             -hip_angle,thigh_angle,calf_angle,
                                             hip_angle,thigh_angle,calf_angle};

    ros::service::waitForService("/gazebo/set_model_state", -1);
    modelStateClient.call(setmodelstate);
    ros::service::waitForService("/gazebo/set_model_configuration", -1);
    if (jointStateClient.call(setjointstate))
    {
        ROS_INFO("BRILLIANT!!!");
        return true;
    } else
    {
        ROS_ERROR("Failed to set joints");
        return false;
    }
}
