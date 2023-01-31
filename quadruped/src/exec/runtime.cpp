/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot runtime used to lanuch the robot
* Author: Zhu Yijie & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       mv implementation of func as runtime.h. @ Zhu Yijie 2021-11-24;
*       add switch mode @ Zhu Linsen 2022-01-30;
*       add robot-runner @ Zhu Yijie 2022-04-01;
*/

#include "exec/runtime.h"

float stairsTime = 13;
float stairsVel = 0.1;

LocomotionController *SetUpController(Robot *quadruped, GaitGenerator* gaitGenerator,
                                    DesiredStateCommand* desiredStateCommand,
                                    StateEstimatorContainer<float>* stateEstimators,
                                    UserParameters* userParameters,
                                    std::string& homeDir)
{    
    ComAdjuster *comAdjuster = new ComAdjuster(quadruped, gaitGenerator, stateEstimators->GetRobotEstimator());
    std::cout << "init comAdjuster finish\n" << std::endl;

    PosePlanner *posePlanner = new PosePlanner(quadruped, gaitGenerator,  stateEstimators);
    std::cout << "init posePlanner finish\n" << std::endl;

    FootholdPlanner *footholdPlanner = new FootholdPlanner(quadruped, gaitGenerator, stateEstimators, userParameters, desiredStateCommand);
    std::cout << "init footholdPlanner finish\n" << std::endl;

    RaibertSwingLegController *swingLegController = new RaibertSwingLegController(quadruped,
                                                                                  gaitGenerator,
                                                                                  stateEstimators,
                                                                                  footholdPlanner,
                                                                                  *userParameters,
                                                                                  homeDir + "config/" + quadruped->robotName
                                                                                    + "/swing_leg_controller.yaml");

    std::cout << "init swingLegController finish\n" << std::endl;
    
    StanceLegControllerInterface *stanceLegController = new StanceLegControllerInterface(quadruped,
                                                                                   gaitGenerator,
                                                                                   stateEstimators,
                                                                                   comAdjuster,
                                                                                   posePlanner,
                                                                                   footholdPlanner,
                                                                                   *userParameters, 
                                                                                   homeDir + "config/" + quadruped->robotName
                                                                                       + "/stance_leg_controller.yaml");

    std::cout << "init stanceLegController finish\n" << std::endl;

    LocomotionController *locomotionController = new LocomotionController(quadruped,
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

void UpdateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed)
{
    controller->swingLegController->desiredSpeed = linSpeed;
    controller->swingLegController->desiredTwistingSpeed = angSpeed;
    controller->stanceLegController->c->desiredSpeed = linSpeed;
    controller->stanceLegController->c->desiredTwistingSpeed = angSpeed;
}


RobotRunner::RobotRunner(Robot* quadrupedIn, std::string& homeDir, ros::NodeHandle& nh)
    : quadruped(quadrupedIn), desiredStateCommand(new DesiredStateCommand(nh, quadruped)), 
    userParameters(homeDir+ "config/user_paramaters.yaml")
{
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
    std::cout << "[RobotRunner] timeStep = " << quadruped->timeStep <<std::endl;
        
    // Action::SitDown(quadruped, 3, 0.001);
    Action::StandUp(quadruped, 3.f, 5.f, 0.001);
    // Action::KeepStand(quadruped, 10,  0.001);
    
    if (quadruped->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
        gaitGenerator = new WalkGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName
                                                        + "/openloop_gait_generator.yaml");
    } else {
        gaitGenerator = new OpenloopGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName
                                                         + "/openloop_gait_generator.yaml");
    }                                                                 
    std::cout << "init gaitGenerator finish\n" << std::endl;
    
    stateEstimators = new StateEstimatorContainer<float>(quadruped, gaitGenerator, &userParameters, 
                                                        "config/" + quadruped->robotName + "/terrain.yaml", 
                                                        homeDir); 
    controlFSM = new ControlFSM<float>(quadruped, stateEstimators, gaitGenerator, desiredStateCommand, &userParameters);    
    resetTime = quadruped->GetTimeSinceReset();
    stateEstimators->Reset(resetTime);
    // gaitGenerator->Reset(resetTime);
    controlFSM->Reset(resetTime);
    stairsVel = userParameters.stairsVel;
    stairsTime = userParameters.stairsTime;
    
}

bool RobotRunner::Update()
{
    // std::cout << "[ROBOT TIME]: " <<std::setprecision(6) << quadruped->GetTimeSinceReset() << std::endl;
    // Run the state estimator step
    // MITTimer estT;
    // todo : the estimators need to be moved outside and runs asynchronously with controllers.
    stateEstimators->Update(); 
    
    // printf("estT SOLVE TIME: %.3f\n", estT.getMs());

    // Find the desired state trajectory
//     bool flag = false;
//     if (desiredStateCommand->count == (stairsTime-2)*1000) {
//          desiredStateCommand->joyCtrlOnRequest = true;
//          desiredStateCommand->gaitSwitch  = true;
//     } else if (desiredStateCommand->count == stairsTime*1000) {
//                 desiredStateCommand->gaitSwitch = true;
//     } else if (desiredStateCommand->joyCtrlOnRequest && desiredStateCommand->count > 26 *1000 && quadruped->baseRollPitchYaw[1]>-0.1) {
//     	desiredStateCommand->gaitSwitch = true;
// 	flag = true;
// //	desiredStateCommand->joyCtrlOnRequest = true;    
//     }
//     if (desiredStateCommand->gaitSwitch) {
// 	if (desiredStateCommand->count >= stairsTime*1000) {
// 		desiredStateCommand->joyCmdVx=stairsVel; 
// 	}
//     }
    // MITTimer commandT;
    desiredStateCommand->Update();
    // printf("commandT SOLVE TIME: %.3f\n", commandT.getMs());
    
    // desiredStateCommand->PrintRawInfo();
    // desiredStateCommand->PrintStateCommandInfo();
    
    // MITTimer contrT;
    controlFSM->RunFSM(hybridAction);
    // for (int i = 0; i < 12; i++) {
    //     hybridAction[i].SetZero();
    
    // }
    // printf("contrT SOLVE TIME: %.3f\n", contrT.getMs());
    // if (flag)
	//     desiredStateCommand->joyCtrlOnRequest = false;
    return true; 
}

bool RobotRunner::Step()
{
    // std::cout << "[robotrunner action] : " << MotorCommand::convertToMatix(hybridAction) << std::endl; 
    // for (int i=0; i<9; i++) {
    //     hybridAction[i] = {0,0,0,0,0};
    // }
    // hybridAction[3] = {0,0,0,0,1.0};
    // hybridAction[4] = {0,0,0,0,0};
    // hybridAction[5] = {0,0,0,0,0};
    // hybridAction[6] = {0,0,0,0,-1.2};
    // hybridAction[7] = {0,0,0,0,0};
    // hybridAction[8] = {0,0,0,0,0};
    // hybridAction[9] = {0,0,0,0,1.0};
    // hybridAction[10] = {0,0,0,0,0};
    // hybridAction[11] = {0,0,0,0,0};
    
    //  for(int legId = 0; legId<4; ++legId) {
    //     if (gaitGenerator->desiredLegState[legId] == LegState::STANCE) {
    //         float phase = gaitGenerator->normalizedPhase[legId];
    //         Vec3<float> angles =  phase* quadruped->standUpMotorAngles.segment(3*legId,3) + (1-phase) * gaitGenerator->firstStanceAngles.segment(3*legId,3);
    //         for(int j=0; j < 3 ; ++j) {
    //             hybridAction[j+3*legId] = {phase *angles[j] + (1-phase)*gaitGenerator->firstStanceAngles[3*legId+j], 100, 0, 2,0};
    //         }   
    //     }
    // }
    // Visualization2D& vis = quadruped->stateDataFlow.visualizer;
    // auto swingController = controlFSM->GetLocomotionController()->GetSwingLegController();
    // Vec3<float> V = quadruped->GetEstimatedVelocityInBaseFrame();
    // auto footPositionB =  quadruped->GetFootPositionsInBaseFrame();
    // auto footPositionW =  quadruped->GetFootPositionsInWorldFrame();
    
    // auto& fullModel = quadruped->model;
    //     auto motorV = quadruped->GetMotorVelocities();
    //     auto motorA = quadruped->GetMotorAngles();
    //     // auto motorddq = quadruped->motorddq;
    //     vis.datax.push_back(quadruped->GetTimeSinceReset());
    //     vis.datay1.push_back(quadruped->basePosition[2]);
    //     vis.datay2.push_back(gaitGenerator->desiredLegState[0]);
    //     vis.datay3.push_back(gaitGenerator->desiredLegState[1]);
    //     vis.datay4.push_back(footPositionB(2, 0));//gaitGenerator->normalizedPhase[0]);
    //     vis.datay5.push_back(V[2]); // fullModel._pGC[Quadruped::linkID::HL][2]
       
    quadruped->Step(MotorCommand::convertToMatix(hybridAction), HYBRID_MODE);
    return 1;
}

RobotRunner::~RobotRunner()
{
    delete quadruped;
    delete gaitGenerator;
    delete stateEstimators;
    delete controlFSM;
}
