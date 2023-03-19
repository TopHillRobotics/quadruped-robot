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

#include "quadruped/exec/qr_runtime.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"
#include "quadruped/action/qr_action.h"

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>

using namespace std;
using namespace Quadruped;

bool ResetRobot(ros::ServiceClient &modelStateClient, ros::ServiceClient &jointStateClient)
{
    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::SetModelConfiguration setjointstate;

    modelState.model_name = "lite2_gazebo";
    modelState.reference_frame = "world";

    geometry_msgs::Twist model_twist;
    geometry_msgs::Pose model_pose;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    model_pose.position.x =0; // 4.5
    model_pose.position.y =0;
    model_pose.position.z =0.3; // 1.3, 0.3
    model_pose.orientation.w = 1;
    model_pose.orientation.x = 0;
    model_pose.orientation.y = 0;
    model_pose.orientation.z = 0;

    modelState.twist = model_twist;
    modelState.pose = model_pose;

    setmodelstate.request.model_state = modelState;
    setjointstate.request.model_name = "lite2_gazebo";
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

void GetComPositionInWorldFrame(Robot* quadruped, ros::ServiceClient& baseStateClient)
{
    gazebo_msgs::GetLinkState gls_request;
    if (baseStateClient.exists()) {
        gls_request.request.link_name = std::string("lite2_gazebo::base");
        gls_request.request.reference_frame=std::string("world");
        // ros::service::waitForService("/gazebo/get_link_state", -1);
        baseStateClient.call(gls_request);
        if (!gls_request.response.success) {
                ROS_INFO("Get Gazebo link state not success!\n");
        }
    } else {
        ROS_INFO("Get Gazebo link state goes wrong!\n");
    }

    const auto & pose_ = gls_request.response.link_state.pose;
    const auto & twist_ = gls_request.response.link_state.twist;
    Vec3<double> posIn = {pose_.position.x, pose_.position.y, pose_.position.z};
    Quat<double> OrientationIn = {pose_.orientation.w,pose_.orientation.x,pose_.orientation.y,pose_.orientation.z};
    Vec3<double> vIn = {twist_.linear.x, twist_.linear.y, twist_.linear.z};

    quadruped->gazeboBasePosition = posIn.cast<float>();
    quadruped->gazeboBaseOrientation = OrientationIn.cast<float>();
    quadruped->gazeboBaseVInBaseFrame = vIn.cast<float>();

    quadruped->gazeboFootPositionInWorldFrame = quadruped->GetFootPositionsInWorldFrame(true, posIn.cast<float>(), OrientationIn.cast<float>());
    // cout << "trueFootPositionInWorldFrame=" << quadruped->gazeboFootPositionInWorldFrame <<endl;
    // std::cout << "gazeboBasePos = " << quadruped->gazeboBasePosition << "\n" <<
    //               "gazeboBaseOri = " << quadruped->gazeboBaseOrientation << std::endl;
}


int main(int argc, char **argv)
{

    //std::string pathToPackage = ros::package::getPath("quadruped");
    //std::string pathToNode =  pathToPackage + ros::this_node::getName();

    std::string homeDir = ros::package::getPath("quadruped") + "/";
    std::string robotName = "lite2_sim";

    ros::init(argc, argv, "lite2sim_standup");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient jointStateClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    bool flag = ResetRobot(modelStateClient, jointStateClient);
    ROS_INFO("Reset the Robot pose");

    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    std::cout << "---------ROS node init finished---------" << std::endl;

    Robot *quadruped = new RobotLite2Sim(nh, privateNh, homeDir + "config/lite2_sim/lite2_sim.yaml");

    std::cout << "robot created........" << std::endl;

    quadruped->Step(Eigen::Matrix<float,5,12>::Zero(), HYBRID_MODE);
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    // ros::Rate loop_rate(round(1.0 / quadruped->timeStep)); // 500--1000 Hz
    ros::Rate loop_rate1(1000);
    ros::Rate loop_rate2(500);
    ROS_INFO("loop rate %f\n", round(1.0 / quadruped->timeStep));

    float standUpTime = 5;
    float totalTime = 10;
    float timeStep = 0.001;

    // standing up
    Timer timer;
    float startTime = timer.GetTimeSinceReset();// robot->GetTimeSinceReset();
    float endTime = startTime + standUpTime;

    Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = quadruped->GetMotorAngles();
    std::cout << "motorAnglesBeforeStandUP: \n" << motorAnglesBeforeStandUP.transpose() << std::endl;
    std::cout << "---------------------Standing Up---------------------" << std::endl;
    std::cout << "robot->standMotorAngles: \n" << quadruped->standUpMotorAngles.transpose() << std::endl;
    for (float t = startTime; t < totalTime; t += timeStep) {
        float blendRatio = (t - startTime) / standUpTime;
        Eigen::Matrix<float, 12, 1> action;
        if (blendRatio < 1.0f) {
            action = blendRatio * quadruped->standUpMotorAngles + (1 - blendRatio) * motorAnglesBeforeStandUP;
            quadruped->Step(action, MotorMode::POSITION_MODE);
            while (timer.GetTimeSinceReset() < t + timeStep) {}
        } else {
            quadruped->Step(action, MotorMode::POSITION_MODE);
            while (timer.GetTimeSinceReset() < t + timeStep) {}
        }
    }

    std::cout << "robot->GetMotorAngles: \n" << quadruped->GetMotorAngles().transpose() << std::endl;
    std::cout << "---------------------Stand Up Finished---------------------" << std::endl;


    // if (count > 20000) {
    //     quadruped->stateDataFlow.visualizer.Show();
    // }

    ros::shutdown();
    return 0;
}
