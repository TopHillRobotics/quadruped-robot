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

#include "quadruped/include/ros/qr_gazebo_controller.h"
#include "quadruped/include/controller/qr_locomotion_controller.h"
#include "quadruped/include/action/qr_action.h"

int main(int argc, char **argv) 
{
    // TODO: get relative path for config.yaml
    std::string homeDir = "";
    std::string robotName = "a1_sim";
    std::string baseLineName = "trot_velocity_motion"
    YAML::Node mainConfig = YAML::LoadFile(homeDir + robotName + baseLineName + "config.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();
    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
    std::vector<std::string> controllerList = mainConfig["controllerList"].as<vector<std::string>>();

    std::cout << "---------Yaml Config Load Finished---------" << std::endl;


    ros::init(argc, argv, "a1_sim");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    stopControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);
    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient jointStateClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ResetRobotByService(modelStateClient, jointStateClient);
    startControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();

    std::cout << "---------Ros Node Init Finished---------" << std::endl;


    qrRobot *quadruped = new qrRobotA1Sim(nh, privateNh, homeDir + robotName + "a1_sim.yaml");
    StandUp(quadruped, 3.f, 5.f, 0.001);
    qrLocomotionController *locomotionController = new qrLocomotionController(quadruped);
    locomotionController->Initialization(homeDir,robotName);
    locomotionController->Reset();
    locomotionController->UpdateDesiredSpeed({0.,0.,0.}, 0.);
    
    std::cout << "---------LocomotionController Init Finished---------" << std::endl;


    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;
    const float MAX_TIME_SECONDS = 300.f;
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();
      
        if (twistMode == TwistMode::ROS) {
            desiredSpeed = cmdVelReceiver->GetLinearVelocity();
            desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity(); 
        }

        locomotionController->UpdateDesiredSpeed(desiredSpeed,desiredTwistingSpeed);
        locomotionController->Update();

        quadruped->Observation();
        auto [hybridAction, qpSol] = locomotionController->GetAction();
        quadruped->ApplyAction(MotorCommand::convertToMatix(hybridAction), MotorMode::HYBRID);


        currentTime = quadruped->GetTimeSinceReset();
        if (abs(quadruped->GetRpy[0]) > 0.5f || abs(quadruped->GetRpy[1]) > 0.5f) {
            ROS_ERROR("The dog is going down, main function exit.");
            break;
        }
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
    }
    
    ROS_INFO("Time is up, end now.");
    ros::shutdown();
    return 0;
}