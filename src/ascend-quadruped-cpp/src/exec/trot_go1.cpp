#include "exec/runtime.h"
//#include <matplotlibcpp.h>
//namespace plt = matplotlibcpp;
using namespace std;
using namespace Quadruped;

Vec4<float> generate_example_linear_angular_speed(float t)
{
    // """Creates an example speed profile based on time for demo purpose."""
    float vx = 0.;
    float vy = 0.;
    float wz = 0.;
    std::vector<float> time_points{0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55,
                                   60, 65, 70, 75, 80, 85, 90};
    std::vector<Vec4<float>> speed_points{{0, 0, 0, 0},
                                          {vx, 0, 0, wz},
                                          {vx, 0, 0, wz},
                                          {0, 0, 0, wz},
                                          {0, 0, 0, wz},
                                          {vx, 0, 0, wz},
                                          {0, 0, 0, wz},
                                          {0, 0, 0, wz},
                                          {0, 0, 0, 0},
                                          {vx, 0, 0, 0},
                                          {0, 0, 0, 0},
                                          {0, 0, 0, wz},
                                          {0, 0, 0, 0},
                                          {vx, 0, 0, 0},
                                          {0, 0, 0, 0},
                                          {0, 0, 0, wz},
                                          {0, 0, 0, 0},
                                          {vx, 0, 0, 0},
                                          {0, 0, 0, 0}};
    std::size_t i = 0;
    for (i = 0; i < time_points.size() - 1; ++i) {
        if (time_points[i + 1] > t) break;
    }

    Vec4<float> speed =
        (speed_points[i + 1] - speed_points[i]) / (time_points[i + 1] - time_points[i]) * (t - time_points[i]) +
            speed_points[i];
    return speed;
}

int ControlProcess(Robot *robot)
{
    std::string homeDir = GetHomeDir();
    std::string robotName = "go1";
    YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/go1/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();

    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();

    std::cout << "---------155---------" << std::endl;

    int argc = 0;
    ros::init(argc, nullptr, "ascend_quadruped_robot");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    ros::Rate loop_rate(1000);
    std::cout << "---------ROS node init finished---------" << std::endl;

    // CmdVelReceiver velReceiver(nh, privateNh);
    // PoseReceiver poseReceiver(nh, privateNh, robot);

    for (int i = 0; i < 1; ++i) {
        robot->ReceiveObservation();
        std::cout << "BaseOrientation:\n" << robot->GetBaseOrientation().transpose() << std::endl;
        std::cout << "MotorAngles:\n" << robot->GetMotorAngles().transpose() << std::endl;
    }
    Action::StandUp(robot, 3, 5, 0.001);
    Action::KeepStand(robot, 2, 0.001);

    LocomotionController *locomotionController = setUpController(robot, homeDir, robotName);
    locomotionController->Reset();
    updateControllerParams(locomotionController, {0., 0.0, 0.}, 0.); // set the linear/angular velocity equal to 0.
    std::cout << "updateControllerParams" << std::endl;
    // ros module init
    RobotOdometryEstimator *legOdom = new RobotOdometryEstimator(robot, locomotionController, nh);
    CmdVelReceiver *cmdVelReceiver = new CmdVelReceiver(nh, privateNh);
    SLAMPoseReceiver *slamPoseReceiver = new SLAMPoseReceiver(nh, privateNh);
    std::cout << "---------ROS Modules Init Finished---------" << std::endl;
    // PosePublisher posePublisher(nh, privateNh, locomotionController->estimator);
    // ros::Timer timer = nh.createTimer(ros::Duration(1.0/20.0), std::bind(&PosePublisher::PoseCallback, posePublisher));

    float startTime = robot->GetTimeSinceReset();
    std::cout << "---------TimeSinceReset: " << startTime << std::endl;

    float currentTime = startTime;
    float startTimeWall = startTime;
    float total_time = 0.f;
    float walkTime = 10.f;
    float endTime = startTime + walkTime;
    int cycles = 1000;
    long long i = 0;
//    std::vector<Vec4<float>> fVec;
    while (ros::ok() &&
        currentTime - startTime < walkTime) {
        startTimeWall = robot->GetTimeSinceReset();
        // Vec4<float> speed = generate_example_linear_angular_speed(startTimeWall - startTime);
        // updateControllerParams(locomotionController, velReceiver.GetLinearVelocity(), velReceiver.GetAngularVelocity());
        // updateControllerParams(locomotionController, speed.head(3), speed[3]);
        if (twistMode == TwistMode::CONST) {
            updateControllerParams(locomotionController,
                                   desiredSpeed,
                                   desiredTwistingSpeed); // const velocity

        } else if (twistMode == TwistMode::ROS) {
            updateControllerParams(locomotionController,
                                   cmdVelReceiver->GetLinearVelocity(),
                                   cmdVelReceiver->GetAngularVelocity()); // ros velocity
        }

        // updateControllerParams(locomotionController, {0.2, 0., 0.}, 0.);
        locomotionController->Update();
        auto[hybridAction, qpSol] = locomotionController->GetAction();
        robot->Step(MotorCommand::convertToMatix(hybridAction), MotorMode::HYBRID_MODE);
        //ros
        legOdom->PublishOdometry();

        currentTime = robot->GetTimeSinceReset();
        total_time += currentTime - startTimeWall;
        i += 1;
        if (i % cycles == 0) {
            printf("time = %f (ms)\n", total_time / cycles * 1000.f);
            total_time = 0;

        }
        if (abs(robot->baseRollPitchYaw[0]) > 0.5f || abs(robot->baseRollPitchYaw[1]) > 0.5f) {
            exit(0);
        }
        while (robot->GetTimeSinceReset() - startTimeWall < robot->timeStep) {}
        ros::spinOnce();
        loop_rate.sleep();
    }
    robot->Step(Eigen::Matrix<float, 5, 12>::Zero(), MotorMode::HYBRID_MODE);
}

int main(int argc, char **argv)
{
    std::string homeDir = GetHomeDir();
    RobotGO1 *robot = new RobotGO1(homeDir + "config/go1/robot_go1.yaml");

    LoopFunc loop_udpSend("udp_send", robot->robotInterface.dt, 2,
                          boost::bind(&RobotInterface::UDPSend, &(robot->robotInterface)));
    LoopFunc loop_udpRecv("udp_recv", robot->robotInterface.dt, 2,
                          boost::bind(&RobotInterface::UDPRecv, &(robot->robotInterface)));
    loop_udpRecv.start();
    loop_udpSend.start();

    Eigen::Matrix<float, 12, 1> motorCommands = Eigen::Matrix<float, 12, 1>::Zero();
    // motorCommands << 0,0.6,-1.2, 0,0.6,-1.2, 0,0.6,-1.2, 0,0.6,-1.2;
    for (int i = 0; i < 100; ++i) {
        robot->ReceiveObservation(); // first receive the imu/motor data.    
        robot->ApplyAction(motorCommands, MotorMode::TORQUE_MODE);
    }
    sleep(3);
    boost::bind(&ControlProcess, robot)();

    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    return 0;
}