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

#include <ros/package.h>

#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"
#include "quadruped/action/qr_action.h"
#include "quadruped/utils/qr_se3.h"

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>


using namespace std;
using namespace Quadruped;


bool ResetRobot(ros::ServiceClient &modelStateClient)
{
    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setmodelstate;

    modelState.model_name = "lite3_gazebo";
    modelState.reference_frame = "world";

    Eigen::Vector3d desired_rpy = Eigen::Vector3d(0, 0, 0);

    auto quat = robotics::math::rpyToQuat(desired_rpy);

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
    model_pose.orientation.w = quat[0];
    model_pose.orientation.x = quat[1];
    model_pose.orientation.y = quat[2];
    model_pose.orientation.z = quat[3];

    modelState.twist = model_twist;
    modelState.pose = model_pose;

    setmodelstate.request.model_state = modelState;

    ros::service::waitForService("/gazebo/set_model_state", -1);
    modelStateClient.call(setmodelstate);
}

void GetComPositionInWorldFrame(qrRobot* quadruped, ros::ServiceClient& baseStateClient)
{
    gazebo_msgs::GetLinkState gls_request;
    if (baseStateClient.exists()) {
        gls_request.request.link_name = std::string("lite3_gazebo::TORSO");
        //gls_request.request.reference_frame=std::string("world");
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

    ros::init(argc, argv, "lite3sim_standup");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();

    std::cout << "---------ROS node init finished---------" << std::endl;

    qrRobot *quadruped = new qrRobotSim(nh, privateNh, "lite3", homeDir);

    ros::ServiceClient baseStateClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation before start the controller: \n" << quadruped->GetBaseOrientation().transpose() << std::endl;



    qrRobotRunner robotRunner(quadruped, homeDir, nh);

    bool flag = ResetRobot(modelStateClient);

    quadruped->ReceiveObservation();

    ros::Rate loop_rate1(1000);
    ros::Rate loop_rate2(500);


    int count = 0;
    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    float avgCost = 0.0f;

    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS && count < 20000) {
        startTimeWall = quadruped->GetTimeSinceReset();

        robotRunner.Update();
        robotRunner.Step();
        //ros
        // legOdom->PublishOdometry();
        // controller2gazeboMsg->PublishGazeboStateCallback();

        currentTime = quadruped->GetTimeSinceReset();
        avgCost += (currentTime - startTimeWall);

        if (count % 3 ==0)
            GetComPositionInWorldFrame(quadruped, baseStateClient);

//        if((count+1) % 1000==0){
//            printf("avg time cost = %f [ms]\n", avgCost);
//            avgCost = 0.;
//        }
//        if(quadruped->basePosition[2] < 0.10
//            || quadruped->stateDataFlow.heightInControlFrame < 0.05
//            || quadruped->basePosition[2]>0.40 || abs(quadruped->baseRollPitchYaw[0]) > 0.6){
//            ROS_ERROR("The dog is going down, main function exit.");
//            cout << "base pos:" << quadruped->basePosition << endl;
//            cout << "base rpy:" << quadruped->GetBaseRollPitchYaw() << endl;
//            // exit(0);
//            break;
//        }

        auto& vis2d = quadruped->stateDataFlow.visualizer;
        auto openloop = robotRunner.GetGaitGenerator();
        vis2d.datax.push_back(count);
        vis2d.datay1.push_back(openloop->phaseInFullCycle[0]);
        vis2d.datay2.push_back(openloop->desiredLegState[0]);
        vis2d.datay3.push_back(openloop->legState[0]);
        vis2d.datay4.push_back(currentTime);

        if(quadruped->useRosTime){
            ros::spinOnce();
            // loop_rate.sleep();
            if (quadruped->timeStep < 0.0015)
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
