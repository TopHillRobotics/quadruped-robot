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

    std::string pathToPackage = ros::package::getPath("a1sim");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();
    std::string robotName = "a1_sim";
    std::string useKeyboard = "0";
    // nh.param<std::string>("usekeyboard", useKeyboard, "n");
    
    // listen the matters of keyboard
    // std::cout << "argc:" << argc << " argv[0]:" << argv[0] << std::endl;
    if(argc == 2){
        std::cout << "argv[1]: " <<argv[1] << std::endl;
        useKeyboard = argv[1];
    }
    qrTeleKeyboard keyboard(nh);
    if(useKeyboard == "1"){
        std::cout << "Keyboard start receving..." << std::endl;
        thread keyboardTh(&qrTeleKeyboard::main, keyboard);
        keyboardTh.detach();
    } else if(useKeyboard == "2"){
        std::cout << "Joy start receving..." << std::endl;
        qrJoy2Twist * msgConvert = new qrJoy2Twist(nh, pathToNode);
    }

    ResetRobotBySystem(nh);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    qrVelocityParamReceiver* cmdVelReceiver = new qrVelocityParamReceiver(nh, pathToNode);
    ros::ServiceClient baseStateClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    std::cout << "---------Ros Module Init finished---------" << std::endl;

    qrRobot *quadruped = new qrRobotA1Sim(nh, pathToNode + "/config/a1_sim.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    Action::StandUp(quadruped, 3.f, 5.f, 0.001);

    qrLocomotionController *locomotionController = setUpController(quadruped, pathToNode, robotName);
    locomotionController->Reset();
    float desiredTwistingSpeed = 0.;
    Eigen::Matrix<float, 3, 1> desiredSpeed = {0.0, 0.0, 0.0};
    updateControllerParams(locomotionController, desiredSpeed, desiredTwistingSpeed);

    std::cout << "---------Locomotion Module Init Finished---------" << std::endl;
    // locomotionController->GetComPositionInWorldFrame(baseStateClient);
    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    std::cout << "----------------Main Loop Starting------------------" << std::endl;

    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();

        desiredSpeed = cmdVelReceiver->GetLinearVelocity();
        desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity();
         
        updateControllerParams(locomotionController,
                                desiredSpeed,
                                desiredTwistingSpeed);

        locomotionController->GetComPositionInWorldFrame(baseStateClient);
        locomotionController->Update();
        auto [hybridAction, qpSol] = locomotionController->GetAction();
        quadruped->Step(qrMotorCommand::convertToMatix(hybridAction), HYBRID_MODE);

        currentTime = quadruped->GetTimeSinceReset();
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
