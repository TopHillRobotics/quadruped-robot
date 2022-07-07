// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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
#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>

#include "quadruped/ros/qr_gazebo_controller.h"
#include "quadruped/controller/qr_locomotion_controller.h"
#include "quadruped/action/qr_action.h"
#include "quadruped/robot/qr_robot_a1sim.h"
#include "quadruped/robot/qr_motor_cmd.h"
#include "quadruped/ros/qr_vel_param_receiver.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "trot_velocity_motion");
    ros::NodeHandle nh;

    std::string pathToPackage = ros::package::getPath("path");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();

    // YAML::Node motionConfig = YAML::LoadFile(pathToNode + "config/motion_config.yaml");
    // int twistMode = motionConfig["speed_update_mode"].as<int>();
    // std::vector<float> linearVel = motionConfig["const_twist"]["linear"].as<std::vector<float >>();
    // auto desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    // auto desiredTwistingSpeed = motionConfig["const_twist"]["angular"].as<float>();
    // std::vector<std::string> controllerList = motionConfig["controllerList"].as<std::vector<std::string>>();

    std::cout << "---------Yaml Config Motion Load Finished---------" << std::endl;

    stopControllers(nh, "/a1_gazebo/controller_manager/switch_controller");
    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient jointStateClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ResetRobotByService(modelStateClient, jointStateClient);
    startControllers(nh, "/a1_gazebo/controller_manager/switch_controller");
    qrVelocityParamReceiver* cmdVelReceiver = new qrVelocityParamReceiver(nh,pathToNode);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    
    std::cout << "---------Ros Node Init Finished---------" << std::endl;


    qrRobot *quadruped = new qrRobotA1Sim(nh, pathToPackage + "/robot_config.yaml");
    StandUp(quadruped, 3.f, 5.f, 0.001);

    float desiredTwistingSpeed = 0.; 
    Eigen::Matrix<float, 3, 1> desiredSpeed = {0.0, 0.0, 0.0};
    qrLocomotionController *locomotionController = new qrLocomotionController(quadruped);
    locomotionController->Initialization(pathToNode + "/config/");
    locomotionController->Reset();
    locomotionController->UpdateDesiredSpeed(desiredSpeed,desiredTwistingSpeed);
    
    std::cout << "---------LocomotionController Init Finished---------" << std::endl;


    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;
    const float MAX_TIME_SECONDS = 300.f;
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();
      
        auto desiredSpeed = cmdVelReceiver->GetLinearVelocity();
        desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity(2);
  
        locomotionController->UpdateDesiredSpeed(desiredSpeed,desiredTwistingSpeed);
        locomotionController->Update();

        quadruped->Observation();
        auto [hybridAction, qpSol] = locomotionController->GetAction();
        quadruped->ApplyAction(qrMotorCmd::CmdsToMatrix5x12(hybridAction), MotorMode::HYBRID);

        currentTime = quadruped->GetTimeSinceReset();
        if (abs(quadruped->GetRobotState()->GetRpy()[0]) > 0.5f || abs(quadruped->GetRobotState()->GetRpy()[1]) > 0.5f) {
            ROS_ERROR("The dog is going down, main function exit.");
            break;
        }
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->GetTimeStep()) {}
    }
    
    ROS_INFO("Time is up, end now.");
    ros::shutdown();
    return 0;
}
