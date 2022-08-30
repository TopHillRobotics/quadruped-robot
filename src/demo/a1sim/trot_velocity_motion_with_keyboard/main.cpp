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

    ros::init(argc, argv, "trot_velocity_motion");
    ros::NodeHandle nh;

    // Get package path.
    std::string pathToPackage = ros::package::getPath("a1sim");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();

    std::string robotName = "a1_sim";

    // Start keyboard receiving thread.
    qrTeleKeyboard keyboard(nh);
    std::cout << "---------Keyboard start receving---------" << std::endl;
    thread keyboardTh(&qrTeleKeyboard::run, keyboard);
    keyboardTh.detach();

    // Reset robot model and gazebo controller.
    ResetRobotBySystem(nh);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();

    // Create command receiver to update velocity from keyboard.
    qrVelocityParamReceiver* cmdVelReceiver = new qrVelocityParamReceiver(nh, pathToNode);
    std::cout << "---------Ros Module Init finished---------" << std::endl;

    // Create the robot.
    qrRobot *quadruped = new qrRobotA1Sim(nh, pathToNode + "/config/a1_sim.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    // Execute the stand up action which is and automic aciton.
    Action::StandUp(quadruped, 3.f, 5.f, 0.001);

    // Create the locomotion controller.
    qrLocomotionController *locomotionController = setUpController(quadruped, pathToNode);
    locomotionController->Reset();
    // Initialize the desired speed of the robot.
    float desiredTwistingSpeed = 0.;
    Eigen::Matrix<float, 3, 1> desiredSpeed = {0.0, 0.0, 0.0};
    updateControllerParams(locomotionController, desiredSpeed, desiredTwistingSpeed);

    std::cout << "---------Locomotion Module Init Finished---------" << std::endl;

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    std::cout << "----------------Main Loop Starting------------------" << std::endl;

    // Start the control loop until the time arrive at MAX_TIME_SECONDS.
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();

        // Update the desired speed if they were changed.
        desiredSpeed = cmdVelReceiver->GetLinearVelocity();
        desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity();
        updateControllerParams(locomotionController,
                                desiredSpeed,
                                desiredTwistingSpeed);

        // Update the locomotion controller include many estimators' update. 
        locomotionController->Update();

        // And compute to get the motor command accord the update.
        auto [hybridAction, qpSol] = locomotionController->GetAction();

        // Execute the motor command accord different control mode(e.g. torque,position,hybrid).
        quadruped->Step(qrMotorCommand::convertToMatix(hybridAction), HYBRID_MODE);

        currentTime = quadruped->GetTimeSinceReset();
        
        // Break if the robot fall down to the ground.
        if (abs(quadruped->GetBaseRollPitchYaw()[0]) > 0.5f || abs(quadruped->GetBaseRollPitchYaw()[1]) > 0.5f) {
            ROS_ERROR("The dog is going down, main function exit.");
            break;
        }
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
    }
    
    ROS_INFO("Time is up, end now.");
    ros::shutdown();
    return 0;
}
