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

#include "mpc_controller/qr_locomotion_controller.h"
namespace Quadruped {
    qrLocomotionController::qrLocomotionController(qrRobot *robotIn,
                                               qrGaitGenerator *gaitGeneratorIn,
                                               qrRobotEstimator *stateEstimatorIn,
                                               qrGroundSurfaceEstimator *groundEstimatorIn,
                                               qrComPlanner *comPlannerIn,
                                               qrSwingLegController *swingLegControllerIn,
                                               qrStanceLegController *stanceLegControllerIn)
    :
        robot(robotIn), gaitGenerator(gaitGeneratorIn), stateEstimator(stateEstimatorIn), groundEstimator(groundEstimatorIn), comPlanner(comPlannerIn),
        swingLegController(swingLegControllerIn), stanceLegController(stanceLegControllerIn)
    {
        resetTime = robot->GetTimeSinceReset();
        timeSinceReset = 0.;
    }

    void qrLocomotionController::Reset()
    {
        resetTime = robot->GetTimeSinceReset();
        timeSinceReset = 0.;
        gaitGenerator->Reset(timeSinceReset);
        stateEstimator->Reset(timeSinceReset);
        groundEstimator->Reset(timeSinceReset);
        comPlanner->Reset(timeSinceReset);
        swingLegController->Reset(timeSinceReset);
        stanceLegController->Reset(timeSinceReset);
    }

    void qrLocomotionController::Update()
    {
        if (!robot->stop) { // not stop = (swingSemaphore > 0) or  (swingSemaphore=0 but not switchToSwing)
            timeSinceReset = robot->GetTimeSinceReset() - resetTime;
        }
        
        // std::cout << "-------locomotion time -------- " << timeSinceReset << std::endl;
        gaitGenerator->Update(timeSinceReset);
        groundEstimator->Update(timeSinceReset);
        stateEstimator->Update(timeSinceReset);
        swingLegController->Update(timeSinceReset);
        stanceLegController->Update(robot->GetTimeSinceReset() - resetTime);
    }

    std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrLocomotionController::GetAction()
    {
        action.clear();
        // Returns the control ouputs (e.g. positions/torques) for all motors. type: map
        auto swingAction = swingLegController->GetAction();
        auto [stanceAction, qpSol] = stanceLegController->GetAction(); // map<int, qrMotorCommand>
        std::vector<qrMotorCommand> action;
        // copy motors' actions from subcontrollers to output variable.         
        for (int joint_id = 0; joint_id < qrRobotConfig::numMotors; ++joint_id) {
            auto it = swingAction.find(joint_id);
            if (it != swingAction.end()) {
                action.push_back(it->second);
            } else {
                action.push_back(stanceAction[joint_id]);
            }
        }
        return {action, qpSol};
    }
} // namespace Quadruped
