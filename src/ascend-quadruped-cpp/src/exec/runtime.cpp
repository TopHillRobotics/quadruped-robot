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

qrLocomotionController *setUpController(Robot *quadruped, std::string homeDir, std::string robotName)
{
    qrGaitGenerator *gaitGenerator;
    gaitGenerator = new qrGaitGenerator(quadruped, homeDir + "config/" + robotName
                                                                         + "/openloop_gait_generator.yaml");
                                                                     
    std::cout << "init gaitGenerator finish\n" << std::endl;

    qrGroundSurfaceEstimator *groundEsitmator = new qrGroundSurfaceEstimator(quadruped, homeDir + "config/" + robotName
                                                                                        + "/terrain.yaml");
    std::cout << "init groundEsitmator finish\n" << std::endl;
    
    RobotEstimator *stateEstimator = new RobotEstimator(quadruped, gaitGenerator, groundEsitmator);
    std::cout << "init robotEstimator finish\n" << std::endl;
     
    qrComPlanner  *comPlanner  = new qrComPlanner (quadruped, gaitGenerator, stateEstimator);
    std::cout << "init comPlanner  finish\n" << std::endl;

    qrPosePlanner *posePlanner = new qrPosePlanner(quadruped, stateEstimator, groundEsitmator, gaitGenerator);
    std::cout << "init posePlanner finish\n" << std::endl;

    qrFootholdPlanner *footholdPlanner = new qrFootholdPlanner(quadruped, groundEsitmator);
    std::cout << "init footholdPlanner finish\n" << std::endl;

    qrSwingLegController *swingLegController = new qrSwingLegController(quadruped,
                                                                                  gaitGenerator,
                                                                                  stateEstimator,
                                                                                  groundEsitmator,
                                                                                  footholdPlanner,
                                                                                  desiredSpeed,
                                                                                  desiredTwistingSpeed,
                                                                                  quadruped->config->bodyHeight,
                                                                                  0.01f,
                                                                                  homeDir + "config/" + robotName
                                                                                    + "/swing_leg_controller.yaml");

    std::cout << "init swingLegController finish\n" << std::endl;

    qrStanceLegController *stanceLegController = new qrStanceLegController(quadruped,
                                                                                   gaitGenerator,
                                                                                   stateEstimator,
                                                                                   groundEsitmator,
                                                                                   comPlanner ,
                                                                                   posePlanner,
                                                                                   footholdPlanner,
                                                                                   desiredSpeed,
                                                                                   desiredTwistingSpeed,
                                                                                   quadruped->config->bodyHeight,
                                                                                   RobotConfig::numLegs,
                                                                                   homeDir + "config/" + robotName
                                                                                       + "/stance_leg_controller.yaml");

    std::cout << "init stanceLegController finish\n" << std::endl;

    qrLocomotionController *locomotionController = new qrLocomotionController(quadruped,
                                                                          gaitGenerator,
                                                                          stateEstimator,
                                                                          groundEsitmator,
                                                                          comPlanner ,
                                                                          posePlanner,
                                                                          swingLegController,
                                                                          stanceLegController);

    std::cout << "init locomotionController finish\n" << std::endl;

    return locomotionController;

}

void updateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed)
{
    controller->swingLegController->desiredSpeed = linSpeed;
    controller->swingLegController->desiredTwistingSpeed = angSpeed;
    controller->stanceLegController->desiredSpeed = linSpeed;
    controller->stanceLegController->desiredTwistingSpeed = angSpeed;
}
