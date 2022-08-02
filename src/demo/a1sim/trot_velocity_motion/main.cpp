
#include "quadruped/exec/runtime.h"
#include "quadruped/sim/a1_sim.h"
#include "quadruped/ros/qr_gazebo_controller_manager.h"
using namespace std;
using namespace Quadruped;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "trot_velocity_motion");
    ros::NodeHandle nh;

    std::string pathToPackage = ros::package::getPath("a1sim");
    std::string pathToNode =  pathToPackage + ros::this_node::getName();
    std::string robotName = "a1_sim";
    stopControllers(nh, "/a1_gazebo/controller_manager/switch_controller");
    ResetRobotBySystem();
    startControllers(nh, "/a1_gazebo/controller_manager/switch_controller");
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    qrVelocityParamReceiver *cmdVelReceiver = new qrVelocityParamReceiver(nh);

    std::cout << "---------ROS node init finished---------" << std::endl;


    YAML::Node mainConfig = YAML::LoadFile(pathToNode + "/config/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();
    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
    qrRobot *quadruped = new qrRobotA1Sim(nh, pathToNode + "/config/a1_sim.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    Action::StandUp(quadruped, 3.f, 5.f, 0.001);

    qrLocomotionController *locomotionController = setUpController(quadruped, pathToNode, robotName);
    locomotionController->Reset();
    updateControllerParams(locomotionController, {0., 0., 0.}, 0.);

    std::cout << "---------LocomotionController Init Finished---------" << std::endl;

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    std::cout << "----------------start control loop------------------" << std::endl;

    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();

        if (twistMode == TwistMode::ROS) {
            desiredSpeed = cmdVelReceiver->GetLinearVelocity();
            desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity(); 
        }
        updateControllerParams(locomotionController,
                                desiredSpeed,
                                desiredTwistingSpeed);
    
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
