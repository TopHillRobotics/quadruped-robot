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

#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"

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

void GetComPositionInWorldFrame(qrRobot* quadruped, ros::ServiceClient& baseStateClient)
{
    gazebo_msgs::GetLinkState gls_request;
    if (baseStateClient.exists()) {
        gls_request.request.link_name = std::string("a1_gazebo::base");
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

    std::string homeDir = ros::package::getPath("quadruped") + "/";
    std::string robotName = "a1_sim";

    ros::init(argc, argv, "a1_sim");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient jointStateClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    bool flag = ResetRobot(modelStateClient, jointStateClient);
    ROS_INFO("Reset the Robot pose");

    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    std::cout << "---------ROS node init finished---------" << std::endl;

    qrRobot *quadruped = new qrRobotA1Sim(nh, privateNh, homeDir + "config/a1_sim/a1_sim.yaml");

    std::cout << "robot created........" << std::endl;

    quadruped->Step(Eigen::Matrix<float,5,12>::Zero(), HYBRID_MODE);
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;
    //
    Visualization2D& vis = quadruped->stateDataFlow.visualizer;
    vis.SetLabelNames({"pitch", "H", "vx in world", "vy in world","vz in world"});

    qrRobotRunner robotRunner(quadruped, homeDir, nh);
    // ros::Rate loop_rate(round(1.0 / quadruped->timeStep)); // 500--1000 Hz
    ros::Rate loop_rate1(1000);
    ros::Rate loop_rate2(500);
    ROS_INFO("loop rate %f\n", round(1.0 / quadruped->timeStep));

    ROS_INFO("LocomotionController Init Finished");
    qrLocomotionController* locomotionController = robotRunner.GetLocomotionController();
    qrStateEstimatorContainer* stateEstimators = robotRunner.GetStateEstimator();
    // ros module init
    ros::ServiceClient baseStateClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    qrController2GazeboMsg *controller2gazeboMsg = new qrController2GazeboMsg(quadruped, locomotionController, nh);
    // SwitchModeReceiver *switchModeReceiver = new SwitchModeReceiver(nh, privateNh);
    ROS_INFO("ROS Modules Init Finished");

    ROS_INFO("TimeSinceReset: %f", quadruped->GetTimeSinceReset());
    GetComPositionInWorldFrame(quadruped, baseStateClient);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;
    Eigen::Matrix<float,12,1> angles = quadruped->GetMotorAngles();
    Eigen::Matrix<float,12,1> stancemotorAngles = angles;
    Eigen::Matrix<float,12,1> motorAnglesAfterKeepStand = angles;
    for (int legId =0; legId<4; ++legId) {
        motorAnglesAfterKeepStand[3*legId + 0] = 0.;
        motorAnglesAfterKeepStand[3*legId + 1] = 1.2;
        motorAnglesAfterKeepStand[3*legId + 2] = -2.4;
    }
    Eigen::Matrix<float,12,1> motorAngles;
    Eigen::Matrix<float, 12, 1> kps, kds;
    kps = quadruped->GetMotorKps();
    kds = quadruped->GetMotorKdp();

    // locomotionController->Update();

    ROS_INFO("start control loop....");
    int switchMode;
    int count = 0;
    float avgCost=0;
    const int n = 10000;

    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();

        // if (count % 3 ==0) {
        //     GetComPositionInWorldFrame(quadruped, baseStateClient);
        //     Vec3<float> robotComRpyRate = quadruped->GetBaseRollPitchYawRate();
        //     Vec4<float> footForces = quadruped->GetFootForce();
        //     Vec3<float> rpy = quadruped->GetBaseRollPitchYaw();
        //     Vec3<float> robotComVelocity = stateEstimators->GetRobotEstimator()->GetEstimatedVelocity();  // base frame
        // }

        robotRunner.Update();
        robotRunner.Step();

        currentTime = quadruped->GetTimeSinceReset();
        avgCost += (currentTime - startTimeWall);
        if ((count+1) % 1000==0) {
            printf("avg time cost = %f [ms]\n", avgCost);
            avgCost = 0.;

        }
        if (quadruped->basePosition[2] < 0.10
            || quadruped->stateDataFlow.heightInControlFrame < 0.05
            || quadruped->basePosition[2]>0.40 || abs(quadruped->baseRollPitchYaw[0]) > 0.6
        ) {
            ROS_ERROR("The dog is going down, main function exit.");
            cout << "base pos:" << quadruped->basePosition << endl;
            cout << "base rpy:" << quadruped->GetBaseRollPitchYaw() << endl;
            // exit(0);
            break;
        }
        // if (count > 80000) {
        //     printf("[268]: count is %d \n", count);
        //     break;
        //     // exit(0);
        // }
        if (quadruped->useRosTime) {
            ros::spinOnce();
            // loop_rate.sleep();
            if (quadruped->timeStep< 0.0015)
                loop_rate1.sleep();
            else
                loop_rate2.sleep();

            // std::cout << "[ros time] = " << ros::Time::now() << std::endl;
        } else {
            while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
        }

        count++;
    }

    quadruped->stateDataFlow.visualizer.Show();

    ros::shutdown();
    return 0;
}
