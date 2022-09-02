#include <typeinfo>

#include "quadruped/exec/runtime.h"
#include "quadruped/robots/qr_robot_a1_sim.h"
#include "quadruped/ros/qr_gazebo_controller_manager.h"

using namespace std;

int main(int argc, char **argv)
{
    // initialize ROS nodes
    ros::init(argc, argv, "demo_helloworld");
    ros::NodeHandle nh;

    // get the node package path
    std::string pathToPackage = ros::package::getPath("a1sim");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();

    std::string robotName = "a1_sim";

    // reset the gazebo controller and robot
    ResetRobotBySystem(nh);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    std::cout << "---------finished: ROS, Gazebo controller and loading robot model---------" << std::endl;

    // create the quadruped robot.
    qrRobot *quadruped = new qrRobotA1Sim(nh, pathToNode + "/config/a1_sim.yaml");
    quadruped->ReceiveObservation();

    /* the quadruped robot stands up.
    (the parameters are robot, the time that stand up need, the total time before excuting other action and time step)
    */
    Action::StandUp(quadruped, 3.f, 5.f, 0.001f);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    // keep ROS running to let the robot stand.
    // 20.f means that the robot keep stands for 20.0 seconds and 0.001 is the time step.
    std::cout << "---------keep quadruped robot standing for 20.0 seconds---------" << std::endl;
    Action::KeepStand(quadruped, 20.f, 0.001f);
    std::cout << "---------quadruped robot sitting down---------" << std::endl;
    Action::SitDown(quadruped, 3.f, 0.001f);
    ROS_INFO("Time is up, end now.");
    ros::shutdown();
    return 0;
}