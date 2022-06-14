/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot runtime used to lanuch the robot
* Author: Zhu Yijie & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       mv implementation of func as runtime.h. @ Zhu Yijie 2021-11-24;
*       add switch mode @ Zhu Linsen 2022-01-30;
*/

#include "exec/runtime.h"

#ifdef _useros
/** @brief a toolkit for loading yaml config file correctly. */
std::string GetHomeDir(std::string homeName)
{
    std::string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find(homeName);
    std::string homeDir = exepath.substr(0, found) + homeName + "src/ascend-quadruped-cpp/";
    std::cout << homeDir << std::endl;
    return homeDir;
}
#else
/** @brief a toolkit for loading yaml config file correctly. */
std::string GetHomeDir(std::string homeName)
{
    std::string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find(homeName);
    std::string homeDir = exepath.substr(0, found) + homeName;
    std::cout << homeDir << std::endl;
    return homeDir;
}
#endif

LocomotionController *setUpController(Robot *quadruped, std::string homeDir, std::string robotName)
{
    OpenloopGaitGenerator *gaitGenerator;
    if (quadruped->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
        gaitGenerator = new WalkGaitGenerator(quadruped, homeDir + "config/" + robotName
                                                                         + "/openloop_gait_generator.yaml");
    } else {
        gaitGenerator = new OpenloopGaitGenerator(quadruped, homeDir + "config/" + robotName
                                                                         + "/openloop_gait_generator.yaml");
    }
                                                                     
    std::cout << "init gaitGenerator finish\n" << std::endl;

    GroundSurfaceEstimator *groundEsitmator = new GroundSurfaceEstimator(quadruped, homeDir + "config/" + robotName
                                                                                        + "/terrain.yaml");
    std::cout << "init groundEsitmator finish\n" << std::endl;
    
    RobotEstimator *stateEstimator = new RobotEstimator(quadruped, gaitGenerator, groundEsitmator);
    std::cout << "init robotEstimator finish\n" << std::endl;
     
    ComAdjuster *comAdjuster = new ComAdjuster(quadruped, gaitGenerator, stateEstimator);
    std::cout << "init comAdjuster finish\n" << std::endl;

    PosePlanner *posePlanner = new PosePlanner(quadruped, stateEstimator, groundEsitmator, gaitGenerator);
    std::cout << "init posePlanner finish\n" << std::endl;

    FootholdPlanner *footholdPlanner = new FootholdPlanner(quadruped, groundEsitmator);
    std::cout << "init footholdPlanner finish\n" << std::endl;

    RaibertSwingLegController *swingLegController = new RaibertSwingLegController(quadruped,
                                                                                  gaitGenerator,
                                                                                  stateEstimator,
                                                                                  groundEsitmator,
                                                                                  footholdPlanner,
                                                                                  desiredSpeed,
                                                                                  desiredTwistingSpeed,
                                                                                  quadruped->bodyHeight,
                                                                                  0.01f,
                                                                                  homeDir + "config/" + robotName
                                                                                    + "/swing_leg_controller.yaml");

    std::cout << "init swingLegController finish\n" << std::endl;

    TorqueStanceLegController *stanceLegController = new TorqueStanceLegController(quadruped,
                                                                                   gaitGenerator,
                                                                                   stateEstimator,
                                                                                   groundEsitmator,
                                                                                   comAdjuster,
                                                                                   posePlanner,
                                                                                   footholdPlanner,
                                                                                   desiredSpeed,
                                                                                   desiredTwistingSpeed,
                                                                                   quadruped->bodyHeight,
                                                                                   quadruped->numLegs,
                                                                                   homeDir + "config/" + robotName
                                                                                       + "/stance_leg_controller.yaml");

    std::cout << "init stanceLegController finish\n" << std::endl;

    LocomotionController *locomotionController = new LocomotionController(quadruped,
                                                                          gaitGenerator,
                                                                          stateEstimator,
                                                                          groundEsitmator,
                                                                          comAdjuster,
                                                                          posePlanner,
                                                                          swingLegController,
                                                                          stanceLegController);

    std::cout << "init locomotionController finish\n" << std::endl;

    return locomotionController;

}

void updateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed)
{
    controller->swingLegController->desiredSpeed = linSpeed;
    controller->swingLegController->desiredTwistingSpeed = angSpeed;
    controller->stanceLegController->desiredSpeed = linSpeed;
    controller->stanceLegController->desiredTwistingSpeed = angSpeed;
}