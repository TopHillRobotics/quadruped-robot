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

#include "controllers/qr_locomotion_controller.h"


namespace Quadruped {

qrLocomotionController::qrLocomotionController(
    qrRobot *robotIn,
    qrGaitGenerator *gaitGeneratorIn,
    qrDesiredStateCommand* desiredStateCommandIn,
    qrStateEstimatorContainer *stateEstimatorIn,
    qrComAdjuster *comAdjusterIn,
    qrPosePlanner *posePlannerIn,
    qrRaibertSwingLegController *swingLegControllerIn,
    qrStanceLegControllerInterface *stanceLegControllerIn,
    qrUserParameters *userParameters):

    robot(robotIn),
    gaitGenerator(gaitGeneratorIn),
    desiredStateCommand(desiredStateCommandIn),
    stateEstimator(stateEstimatorIn),
    comAdjuster(comAdjusterIn),
    posePlanner(posePlannerIn),
    swingLegController(swingLegControllerIn),
    stanceLegController(stanceLegControllerIn)
{
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.;
    BindCommand();
}


void qrLocomotionController::Reset()
{
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.;
    gaitGenerator->Reset(timeSinceReset);
    comAdjuster->Reset(timeSinceReset);
    posePlanner->Reset(timeSinceReset);
    swingLegController->Reset(timeSinceReset);
    stanceLegController->Reset(timeSinceReset);
    BindCommand();
}


void qrLocomotionController::Update()
{
    if (!robot->stop) {
        timeSinceReset = robot->GetTimeSinceReset() - resetTime;
    }

    /* Find the current gait schedule */
    gaitGenerator->Update(timeSinceReset);

    bool switchToSwing = false;
    if (robot->controlParams["mode"]==LocomotionMode::WALK_LOCOMOTION) {
        // for walk mode
        const Vec4<int>& newLegState = gaitGenerator->legState;
        const Vec4<int>& curLegState = gaitGenerator->curLegState;
        for(int legId =0; legId<4; legId++) {
            if((newLegState(legId) == LegState::SWING && curLegState(legId) == LegState::STANCE)
                || newLegState(legId) == LegState::USERDEFINED_SWING) {
                    switchToSwing=true;
                    break;
                }
        }
        if (switchToSwing) {
            if (swingSemaphore > 0) {
                swingSemaphore--;
            } else if (swingSemaphore == 0)
            {
                swingSemaphore--;
                stopTick = robot->GetTimeSinceReset();

                /* When stop, all legs must stay stance if phase gap=0.25. */
                robot->stop = true;
                printf("stop robot!============\n");
                posePlanner->ResetBasePose(timeSinceReset);
            } else { // swingSemaphore == -1
                ;
            }
        }
    }

    switch (robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            comAdjuster->Update(timeSinceReset);
        } break;
        case LocomotionMode::WALK_LOCOMOTION: {
            if (switchToSwing && swingSemaphore >=0) {
                posePlanner->Update(timeSinceReset);
                printf("update pose plan finish\n");
            }
        } break;
        default: {
            // comAdjuster->Update(timeSinceReset);
            break;
        }
    }
    swingLegController->Update(timeSinceReset);
    stanceLegController->Update(timeSinceReset);
}


std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrLocomotionController::GetAction()
{
    action.clear();

    /* Returns the control ouputs (e.g. positions/torques) for all motors. type: map */
    auto swingAction = swingLegController->GetAction();

    auto [stanceAction, qpSol] = stanceLegController->GetAction(); // map<int, MotorCommand>

    std::vector<qrMotorCommand> action;
    for (int joint_id = 0; joint_id < NumMotor; ++joint_id) {
        auto it = swingAction.find(joint_id);
        if (it != swingAction.end()) {
            action.push_back(it->second);
        } else {
            action.push_back(stanceAction[joint_id]);
        }
    }
    return {action, qpSol};
}
    

std::tuple<std::vector<qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrLocomotionController::GetFakeAction()
{
    action.clear();
    Eigen::Matrix<float, 3, 4> qpSol = Eigen::Matrix<float, 3, 4>::Zero();
    std::vector<qrMotorCommand> action;
    /* Copy motors' actions from subcontrollers to output variable. */
    for (int joint_id = 0; joint_id < NumMotor; ++joint_id) {
        action.push_back({0,0,0,0,0});
    }
    return {action, qpSol};
}

} // Namespace Quadruped
