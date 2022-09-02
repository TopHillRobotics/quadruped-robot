#include <thread>
#include <typeinfo>

#include "quadruped/exec/runtime.h"
#include "ros/qr_msg_convert.h"
#include "quadruped/ros/qr_telekeyboard.h"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/ros/qr_gazebo_controller_manager.h"
using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "demo_helloworld");
    ros::NodeHandle nh;

    // Get package path.
    std::string pathToPackage = ros::package::getPath("a1sim");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();

    std::string robotName = "a1_sim";

    // Reset robot model and gazebo controller.
    ResetRobotBySystem(nh);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    std::cout << "---------Ros Module Init finished---------" << std::endl;

    // Create the robot.
    qrRobot *quadruped = new qrRobotA1Sim(nh, pathToNode + "/config/a1_sim.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    // Execute the stand up action which is and automic aciton.
    Action::StandUp(quadruped, 3.f, 5.f, 0.001);

    std::cout << "---------Locomotion Module Init Finished---------" << std::endl;
    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    std::cout << "----------------Main Loop Starting------------------" << std::endl;

    // Keep program running to maintain the robot stand.
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        /*
          Put control logic here.
        */
    }
    
    ROS_INFO("Time is up, end now.");
    ros::shutdown();
    return 0;
}
