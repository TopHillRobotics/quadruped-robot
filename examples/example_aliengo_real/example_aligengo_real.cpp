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

#include "exec/runtime.h"
#include "ros/control2gazebo_msg.h"

using namespace std;
using namespace Quadruped;

int main(int argc, char **argv)
{
    std::string homeDir = GetHomeDir();
    std::string robotName = "aliengo";
    YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/aliengo/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    
    ros::init(argc, argv, "ascend_quadruped_robot");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    std::cout << "---------ROS node init finished---------" << std::endl;

    Robot *quadruped = new RobotA1(homeDir + "config/aliengo/aliengo_robot.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;
    RobotRunner robotRunner(quadruped, homeDir, nh);

    ros::Rate loop_rate1(1000);
    ros::Rate loop_rate2(500);
    
    LocomotionController* locomotionController = robotRunner.GetLocomotionController();
    StateEstimatorContainer<float>* stateEstimators = robotRunner.GetStateEstimator();
    DesiredStateCommand* desiredStateCommand = robotRunner.GetDesiredStateCommand();
    auto& vis = quadruped->stateDataFlow.visualizer;
    // ros module init
    // RobotOdometryEstimator *legOdom = new RobotOdometryEstimator(quadruped, nh);
    // CmdVelReceiver *cmdVelReceiver = new CmdVelReceiver(nh, privateNh);
    // SLAMPoseReceiver *slamPoseReceiver = new SLAMPoseReceiver(nh, privateNh);
    // SwitchModeReceiver *switchModeReceiver = new SwitchModeReceiver(nh, privateNh);
    // ROS_INFO("ROS Modules Init Finished");
    // Controller2GazeboMsg *controller2gazeboMsg = new Controller2GazeboMsg(quadruped, locomotionController, nh);
    ROS_INFO("ROS Modules Init Finished");
    
    float startTime = quadruped->GetTimeSinceReset();
    ROS_INFO("TimeSinceReset: %f", startTime);
    
    float currentTime = startTime;
    float startTimeWall = startTime;
    float avgCost = 0.f;
    int count = 0;
    int switchMode;
    ROS_INFO("start control loop....");
    Vec3<float> desiredSpeed(0,0,0);
    float desiredTwistingSpeed = 0;
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();
        // switchMode = switchModeReceiver->GetSwitchMode();        
        // if (desiredStateCommand->rosCmdRequest) {
        //     desiredSpeed = cmdVelReceiver->GetLinearVelocity();
        //     desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity();
        //     desiredStateCommand->filteredVel = desiredSpeed;
        //     desiredStateCommand->filteredOmega << 0, 0, desiredTwistingSpeed;
        // }
        // if (switchMode != 2 && quadruped->controlParams["mode"] != switchMode) {
        //     SwitchMode(quadruped, locomotionController, desiredSpeed, desiredTwistingSpeed, switchMode, startTimeWall);
        // }
        // UpdateControllerParams(locomotionController,
        //                         desiredSpeed,
        //                            desiredTwistingSpeed);
        // locomotionController->Update();
        // auto[hybridAction, qpSol] = locomotionController->GetAction();
        // quadruped->Step(MotorCommand::convertToMatix(hybridAction), HYBRID_MODE);
        robotRunner.Update();
        robotRunner.Step();

        //ros
        // legOdom->PublishOdometry();
        // controller2gazeboMsg->PublishGazeboStateCallback();

        currentTime = quadruped->GetTimeSinceReset();
        avgCost += (currentTime - startTimeWall);
            
        if ((count+1) % 1000==0) {
            printf("avg time cost = %f [ms]\n", avgCost);
            avgCost = 0.;
        }
        
        if (quadruped->stateDataFlow.heightInControlFrame < 0.05
            // quadruped->basePosition[2]<0.1 
            // || quadruped->basePosition[0] > 1.5
            || quadruped->basePosition[2]>0.5 || abs(quadruped->baseRollPitchYaw[0]) > 0.8f 
            || abs(quadruped->baseRollPitchYaw[1]) > 0.8f) {
            printf("[main] exit(0)\n");
            cout << "H in Control Frame = " << quadruped->stateDataFlow.heightInControlFrame << endl;
            cout << "base pos:" << quadruped->basePosition<<endl;
            cout << "base rpy:" << quadruped->baseRollPitchYaw<<endl;
            break;
        }
        if (count >= 1000000-1) {
            printf("[268]: count is %d \n", count);
            break;
        }

        ros::spinOnce();
        // loop_rate.sleep();
        if (quadruped->timeStep< 0.0015)
            loop_rate1.sleep();
        else 
            loop_rate2.sleep();
        // ros::Rate(round(1.0 / quadruped->timeStep)).sleep(); // 500--1000 Hz
    	if (!quadruped->useRosTime) {
	        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
	    }
        count++;
    }

    quadruped->Step(Eigen::Matrix<float, 5, 12>::Zero(), MotorMode::HYBRID_MODE);
    if (count > 3000) {
        for (int i=0; i <=3; ++i) {
            vis.sa[i].PrintStatistics();
        }
        // vis.Show();
    }

    ros::shutdown();
    return 0;
}

