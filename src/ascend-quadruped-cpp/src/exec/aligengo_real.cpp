#include "exec/runtime.h"

using namespace std;
using namespace Quadruped;

int main(int argc, char **argv)
{
    std::string homeDir = GetHomeDir();
    std::string robotName = "aliengo";
    YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/aliengo/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();

    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();

    ros::init(argc, argv, "ascend_quadruped_robot");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    ros::Rate loop_rate(1000);
    std::cout << "---------ROS node init finished---------" << std::endl;

    Robot *quadruped = new RobotA1(homeDir + "config/aliengo/aliengo_robot.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    //    Action::SitDown(quadruped, 3, 0.001);
    Action::StandUp(quadruped, 2.f, 3.f, 0.001);

    LocomotionController *locomotionController = setUpController(quadruped, homeDir, robotName);
    std::cout << "---------LocomotionController Init Finished---------" << std::endl;
    locomotionController->Reset();
    std::cout << "---------LocomotionController Reset Finished---------" << std::endl;

    // ros module init
    RobotOdometryEstimator *legOdom = new RobotOdometryEstimator(quadruped, locomotionController, nh);
    CmdVelReceiver *cmdVelReceiver = new CmdVelReceiver(nh, privateNh);
    SLAMPoseReceiver *slamPoseReceiver = new SLAMPoseReceiver(nh, privateNh);
    SwitchModeReceiver *switchModeReceiver = new SwitchModeReceiver(nh, privateNh);
    std::cout << "---------ROS Modules Init Finished---------" << std::endl;

    std::cout << "---------TimeSinceReset: " << quadruped->GetTimeSinceReset() << std::endl;

    updateControllerParams(locomotionController, {0., 0., 0.}, 0.);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;
    int switchMode;
    std::cout << "------------------start control loop------------------ " << std::endl;
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();
        switchMode = switchModeReceiver->GetSwitchMode();        
        if (twistMode == TwistMode::ROS) {
            desiredSpeed = cmdVelReceiver->GetLinearVelocity();
            desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity();
        }
        if (switchMode != 2 && quadruped->controlParams["mode"] != switchMode) {
            SwitchMode(quadruped, locomotionController, desiredSpeed, desiredTwistingSpeed, switchMode, startTimeWall);
        }
        updateControllerParams(locomotionController,
                                desiredSpeed,
                                   desiredTwistingSpeed);
        locomotionController->Update();
        auto[hybridAction, qpSol] = locomotionController->GetAction();
        quadruped->Step(MotorCommand::convertToMatix(hybridAction), HYBRID_MODE);

        //ros
        legOdom->PublishOdometry();

        currentTime = quadruped->GetTimeSinceReset();
//        std::cout << "cycle time: " << (currentTime - startTimeWall) * 1000.f << " ms" << std::endl;
        if (abs(quadruped->baseRollPitchYaw[0]) > 0.5f || abs(quadruped->baseRollPitchYaw[1]) > 0.5f) {
            exit(0);
        }

        ros::spinOnce();
        loop_rate.sleep();
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
//        std::cout << ros::Time::now() << endl;
    }

    return 0;
}
