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

#include "exec/runtime.h"

qrLocomotionController *setUpController(qrRobot *quadruped, std::string homeDir, ros::NodeHandle &nh, bool useMPC)
{
    qrGaitGenerator *gaitGenerator;
    bool isSim;
    nh.getParam("isSim", isSim);
    std::string prefix = isSim ? "sim" : "real";
    gaitGenerator = new qrGaitGenerator(quadruped, homeDir + "/" + prefix + "_config/openloop_gait_generator.yaml");
                                                                     
    std::cout << "init gaitGenerator finish\n" << std::endl;

    qrGroundSurfaceEstimator *groundEsitmator = new qrGroundSurfaceEstimator(quadruped, homeDir + "/" + prefix + "_config/terrain.yaml");
    std::cout << "init groundEsitmator finish\n" << std::endl;
    
    qrRobotEstimator *stateEstimator = new qrRobotEstimator(quadruped, gaitGenerator, groundEsitmator);
    std::cout << "init robotEstimator finish\n" << std::endl;
     
    qrComPlanner  *comPlanner  = new qrComPlanner (quadruped, gaitGenerator, stateEstimator);
    std::cout << "init comPlanner  finish\n" << std::endl;

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
                                                                        homeDir + "/" + prefix + "_config/swing_leg_controller.yaml");

    std::cout << "init swingLegController finish\n" << std::endl;

    qrStanceLegController *stanceLegController = qrStanceLegController::createStanceController(
          quadruped,
          gaitGenerator,
          stateEstimator,
          groundEsitmator,
          comPlanner,
          footholdPlanner,
          desiredSpeed,
          desiredTwistingSpeed,
          quadruped->config->bodyHeight,
          qrRobotConfig::numLegs,
          homeDir + "/" + prefix + "_config/stance_leg_controller.yaml",
          std::vector<float>{0.45f, 0.45f, 0.45f, 0.45f},
          useMPC);
    std::cout << "init stanceLegController finish\n" << std::endl;

    qrLocomotionController *locomotionController = new qrLocomotionController(quadruped,
                                                                          gaitGenerator,
                                                                          stateEstimator,
                                                                          groundEsitmator,
                                                                          comPlanner,
                                                                          swingLegController,
                                                                          stanceLegController);

    std::cout << "init locomotionController finish\n" << std::endl;

    return locomotionController;
}

void updateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed)
{
    controller->swingLegController->UpdateControlParameters(linSpeed, angSpeed);
    controller->stanceLegController->UpdateControlParameters(linSpeed, angSpeed);
}
