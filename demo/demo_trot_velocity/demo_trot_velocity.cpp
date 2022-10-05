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

#include "quadruped/exec/runtime.h"
#include "quadruped/robots/qr_robot_sim.h"
#include "quadruped/robots/qr_robot_real.h"
#include "quadruped/ros/qr_gazebo_controller_manager.h"

int main(int argc, char **argv)
{
    // initialize ROS nodes
    ros::init(argc, argv, "demo_trot_velocity");
    ros::NodeHandle nh;
    
    // use AsyncSpinner rather than standard spinner
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    
    // get the node package path
    std::string pathToPackage = ros::package::getPath("demo");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();
    std::string robotName = "a1";
    qrRobot *quadruped;

    nh.setParam("isSim", true);

    if(argc == 1 || (argc == 2 && std::string(argv[1]) == "sim")) {
        nh.getParam("robotName", robotName);

        // reset the gazebo controller and robot
        ResetRobotBySystem(nh, robotName);

        ROS_INFO("---------finished: ROS, Gazebo controller and loading robot model---------");
        
        // create a quadruped robot.
        quadruped = new qrRobotSim(nh, robotName, LocomotionMode::VELOCITY_LOCOMOTION);
    
    } else if(argc == 2 && std::string(argv[1]) == "real"){
        nh.setParam("isSim", false);
        quadruped = new qrRobotReal(robotName, LocomotionMode::VELOCITY_LOCOMOTION);
    }
    
    // create a velocity parameter receiver to receive any velocity change
    qrVelocityParamReceiver* cmdVelReceiver = new qrVelocityParamReceiver(nh, pathToNode);
    
    
    quadruped->ReceiveObservation();

    // stands up (the parameters are robot, the time for standing up, 
    // the total time before excuting other actions, the time step)
    Action::StandUp(quadruped, 3.f, 5.f, 0.001);

    // create a locomotion controller
    qrLocomotionController *locomotionController = setUpController(quadruped, pathToNode,nh);
    locomotionController->Reset();
    
    // initialize the desired speed
    float desiredTwistingSpeed = 0.;
    Eigen::Matrix<float, 3, 1> desiredSpeed = {0.0, 0.0, 0.0};
    
    updateControllerParams(locomotionController, desiredSpeed, desiredTwistingSpeed);

    std::cout << "---------Locomotion initialization is finished---------" << std::endl;

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    std::cout << "----------------Main Loop Starting------------------" << std::endl;

    // start the control loop until the time arrive at MAX_TIME_SECONDS.
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        
        startTimeWall = quadruped->GetTimeSinceReset();

        // update the desired speed if changed.
        desiredSpeed = cmdVelReceiver->GetLinearVelocity();
        desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity();
        
        updateControllerParams(locomotionController, desiredSpeed, desiredTwistingSpeed);

        // update the locomotion controller and estimators
        locomotionController->Update();

        // compute the motor commands
        auto [hybridAction, qpSol] = locomotionController->GetAction();

        // execute the motor command in a given control mode (e.g. torque, position, hybrid).
        quadruped->Step(qrMotorCommand::convertToMatix(hybridAction), HYBRID_MODE);

        currentTime = quadruped->GetTimeSinceReset();
        
        // stop if the robot falls (roll>0.5 or pitch>0.5)
        if (abs(quadruped->GetBaseRollPitchYaw()[0]) > 0.5f || abs(quadruped->GetBaseRollPitchYaw()[1]) > 0.5f) {
            ROS_ERROR("The dog is falling!");
            break;
        }

        // wait until this step has synchronizing frequency.
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
    }
    
    ros::shutdown();
    return 0;
}
