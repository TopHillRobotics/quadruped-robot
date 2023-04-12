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

#include "exec/qr_robot_runner.h"


float stairsTime = 13;
float stairsVel = 0.1;


qrLocomotionController *SetUpController(
    qrRobot *quadruped,
    qrGaitGenerator* gaitGenerator,
    qrDesiredStateCommand* desiredStateCommand,
    qrStateEstimatorContainer* stateEstimators,
    qrUserParameters* userParameters,
    std::string& homeDir)
{    
    qrComAdjuster *comAdjuster = new qrComAdjuster(quadruped, gaitGenerator, stateEstimators->GetRobotEstimator());
    std::cout << "init comAdjuster finish\n" << std::endl;

    qrPosePlanner *posePlanner = new qrPosePlanner(quadruped, gaitGenerator,  stateEstimators);
    std::cout << "init posePlanner finish\n" << std::endl;

    qrFootholdPlanner *footholdPlanner = new qrFootholdPlanner(quadruped, gaitGenerator, stateEstimators, userParameters, desiredStateCommand);
    std::cout << "init footholdPlanner finish\n" << std::endl;

    qrRaibertSwingLegController *swingLegController = new qrRaibertSwingLegController(quadruped,
                                                                                  gaitGenerator,
                                                                                  stateEstimators,
                                                                                  footholdPlanner,
                                                                                  *userParameters,
                                                                                  homeDir + "config/" + quadruped->robotName
                                                                                    + "/swing_leg_controller.yaml");

    std::cout << "init swingLegController finish\n" << std::endl;
    
    qrStanceLegControllerInterface *stanceLegController = new qrStanceLegControllerInterface(quadruped,
                                                                                   gaitGenerator,
                                                                                   stateEstimators,
                                                                                   comAdjuster,
                                                                                   posePlanner,
                                                                                   footholdPlanner,
                                                                                   *userParameters, 
                                                                                   homeDir + "config/" + quadruped->robotName
                                                                                       + "/stance_leg_controller.yaml");

    std::cout << "init stanceLegController finish\n" << std::endl;

    qrLocomotionController *locomotionController = new qrLocomotionController(quadruped,
                                                                          gaitGenerator,
                                                                          desiredStateCommand,
                                                                          stateEstimators,
                                                                          comAdjuster,
                                                                          posePlanner,
                                                                          swingLegController,
                                                                          stanceLegController,
                                                                          userParameters);

    std::cout << "init locomotionController finish\n" << std::endl;

    return locomotionController;

}


void UpdateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed)
{
    controller->swingLegController->desiredSpeed = linSpeed;
    controller->swingLegController->desiredTwistingSpeed = angSpeed;
    controller->stanceLegController->c->desiredSpeed = linSpeed;
    controller->stanceLegController->c->desiredTwistingSpeed = angSpeed;
}


qrRobotRunner::qrRobotRunner(qrRobot* quadrupedIn, std::string& homeDir, ros::NodeHandle& nh):
    quadruped(quadrupedIn),
    desiredStateCommand(new qrDesiredStateCommand(nh, quadruped)),
    userParameters(homeDir+ "config/user_parameters.yaml")
{
    std::cout <<"[Runner] name: "  << quadruped->robotName <<std::endl;
    std::cout << homeDir + "config/" + quadruped->robotName + "/main.yaml" << std::endl;
    YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/" + quadruped->robotName + "/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    std::vector<float> linearVel = mainConfig["const_twist"]["linear"].as<std::vector<float >>();
    Vec3<float> desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    float desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
    Eigen::MatrixXf::Map(&userParameters.desiredSpeed[0], 3, 1) = desiredSpeed;
    userParameters.desiredTwistingSpeed = desiredTwistingSpeed;
    desiredStateCommand->vDesInBodyFrame = desiredSpeed;
    desiredStateCommand->wDesInBodyFrame << 0,0, desiredTwistingSpeed;
    quadruped->timeStep = 1.0 / userParameters.controlFrequency;
    
    quadruped->ReceiveObservation();
    quadruped->ReceiveObservation();
    quadruped->ReceiveObservation();
    // Action::SitDown(quadruped, 3, 0.001);
    Action::StandUp(quadruped, 4.f, 5.f, 0.001);
    // Action::KeepStand(quadruped, 10,  0.001);
    //Action::ControlFoot(quadruped, nullptr, 15, 0.001);
    
    if (quadruped->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
        gaitGenerator = new qrWalkGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName
                                                        + "/openloop_gait_generator.yaml");
    } else {
        gaitGenerator = new qrOpenLoopGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName
                                                         + "/openloop_gait_generator.yaml");
    }                                                                 
    std::cout << "init gaitGenerator finish\n" << std::endl;
    
    stateEstimators = new qrStateEstimatorContainer(quadruped, gaitGenerator, &userParameters, 
                                                        "config/" + quadruped->robotName + "/terrain.yaml", 
                                                        homeDir); 
    controlFSM = new qrControlFSM<float>(quadruped, stateEstimators, gaitGenerator, desiredStateCommand, &userParameters);    
    resetTime = quadruped->GetTimeSinceReset();
    stateEstimators->Reset();
    // gaitGenerator->Reset(resetTime);
    controlFSM->Reset(resetTime);
    stairsVel = userParameters.stairsVel;
    stairsTime = userParameters.stairsTime;
    
}


bool qrRobotRunner::Update()
{
    stateEstimators->Update(); 
    
    desiredStateCommand->Update();

    controlFSM->RunFSM(hybridAction);

    return true; 
}


bool qrRobotRunner::Step()
{
    Visualization2D& vis = quadruped->stateDataFlow.visualizer;
    auto swingController = controlFSM->GetLocomotionController()->GetSwingLegController();
    auto torqueController = controlFSM->GetLocomotionController()->GetStanceLegController();

    Vec4<float> f = quadruped->GetFootForce();
    Vec3<float> w = quadruped->GetBaseRollPitchYawRate();
    Vec3<float> rpy = quadruped->GetBaseRollPitchYaw();
    Vec3<float> V = quadruped->GetBaseVelocityInBaseFrame();
    auto footPositionB =  quadruped->GetFootPositionsInBaseFrame();
    // auto footPositionW =  quadruped->GetFootPositionsInWorldFrame();
    // auto& fullModel = quadruped->model;
    auto motorV = quadruped->GetMotorVelocities();
    auto motorA = quadruped->GetMotorAngles();
    auto foot_pos_target_last_time = swingController->foot_pos_target_last_time;
    auto mpcContacts = torqueController->contacts;
    // auto motorddq = quadruped->motorddq;
    float t = quadruped->GetTimeSinceReset();
    
    quadruped->Step(qrMotorCommand::convertToMatix(hybridAction), HYBRID_MODE);
    return 1;
}


qrRobotRunner::~qrRobotRunner()
{
    delete quadruped;
    delete gaitGenerator;
    delete stateEstimators;
    delete controlFSM;
}
