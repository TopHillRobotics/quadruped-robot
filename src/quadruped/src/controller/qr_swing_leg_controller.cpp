// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: Xinyu Zhang   email: tophill.robotics@gmail.com

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

#include "controller/qr_swing_leg_controller.h"

QuadrupedRobot::qrSwingLegController::
    qrSwingLegController(qrRobot *robot,
                         qrOpenloopGaitGenerator *gaitGenerator,
                         qrRobotEstimator *stateEstimator,
                         qrGroundSurfaceEstimator *groundEstimator,
                         qrFootholdPlanner *footholdPlanner,
                         Eigen::Matrix<float, 3, 1> desiredSpeed,
                         float desiredTwistingSpeed,
                         float desiredHeight,
                         float footClearance,
                         std::string configPath):robot(robot),
                                                 gaitGenerator(gaitGenerator),
                                                 robotEstimator(stateEstimator),
                                                 groundEstimator(groundEstimator),
                                                 footholdPlanner(footholdPlanner),
                                                 desiredSpeed(desiredSpeed),
                                                 desiredTwistingSpeed(desiredTwistingSpeed),
                                                 configFilepath(configPath)
{
    this->desiredHeight = Matrix<float, 3, 1>(0, 0, desiredHeight - footClearance);
    YAML::Node swingLegConfig = YAML::LoadFile(configPath);
    this->footInitPose = swingLegConfig["swing_leg_params"]["foot_in_world"].as<std::vector<std::vector<float>>>();
}

QuadrupedRobot::qrSwingLegController::Reset()
{
    this->phaseSwitchFootLocalPos = robot->GetFootPositionsInBaseFrame();
    this->phaseSwitchFootGlobalPos = robot->GetFootPositionsInWorldFrame();
    Matrix<float, 1, 4> footX = MatrixXf::Map(&footInitPose[0][0], 1, 4);
    Matrix<float, 1, 4> footY = MatrixXf::Map(&footInitPose[1][0], 1, 4);
    Matrix<float, 1, 4> footZ = MatrixXf::Map(&footInitPose[2][0], 1, 4);      
    
    this->footHoldInWorldFrame.row(0) << 0.185f, 0.185f, -0.175f, -0.175f;
    this->footHoldInWorldFrame.row(1) << -0.145f, 0.145f, -0.145f, 0.145f;
    this->footHoldInWorldFrame.row(2) << 0.f, 0.f, 0.f, 0.f;
    this->footHoldInWorldFrame(0, 0) -= 0.05;
    this->footHoldInWorldFrame(0, 3) -= 0.05;
    
    switch (robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << this->phaseSwitchFootLocalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << this->phaseSwitchFootGlobalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << this->footHoldInWorldFrame << std::endl;
        } break;
        case LocomotionMode::WALK_LOCOMOTION: {
            this->footHoldInWorldFrame = this->phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
            std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << this->phaseSwitchFootLocalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << this->phaseSwitchFootGlobalPos
                    << std::endl;
            std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << this->footHoldInWorldFrame << std::endl;
        } break;
        default: break;  
    }

    swingJointAnglesVelocities.clear();
}

QuadrupedRobot::qrSwingLegController::Update()
{
    const Vec4<int>& newLegState = this->gaitGenerator->desiredLegState;
    const Vec4<int>& curLegState = this->gaitGenerator->curLegState;
    // the footHoldOffset is first init at robot.h, then update it at groundEstimator.cpp 
    Eigen::Matrix<float, 3, 1> constOffset = {this->robot->footHoldOffset, 0.f, 0.f};
    
    // Detects phase switch for each leg so we can remember the feet position at
    // the beginning of the swing phase.
    switch (this->robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            for (int legId = 0; legId < 4; ++legId) {
                if ((newLegState(legId) == LegState::SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                    && curLegState(legId) == LegState::STANCE 
                    && !this->robot->stop) {
        
                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
                    this->phaseSwitchFootGlobalPos.col(legId) = this->robot->GetFootPositionsInWorldFrame().col(legId);
                    if (legId == 0) { //update four leg footholds
                        this->footholdPlanner->UpdateOnce(footHoldInWorldFrame); // based on the last foothold position
                    }
                    this->footHoldInWorldFrame.col(legId) += this->footholdPlanner->GetFootholdsOffset().col(legId);
                }
            }
        } break;
        case LocomotionMode::WALK_LOCOMOTION: {
            for (int legId = 0; legId < 4; ++legId) {
                if ((newLegState(legId) == SubLegState::TRUE_SWING || newLegState(legId) == LegState::USERDEFINED_SWING)
                    && curLegState(legId) == SubLegState::UNLOAD_FORCE 
                    && !this->robot->stop) {
                    this->phaseSwitchFootLocalPos.col(legId) = this->robot->GetFootPositionsInBaseFrame().col(legId);
                    this->phaseSwitchFootGlobalPos.col(legId) = this->robot->GetFootPositionsInWorldFrame().col(legId);
                    // case 1:
                    this->footholdPlanner->UpdateOnce(footHoldInWorldFrame, {legId});
                    this->footHoldInWorldFrame.col(legId) = this->footholdPlanner->GetFootholdInWorldFrame(legId);
                    // case 2:
                    Vec3<float> footSourcePosition;
                    Vec3<float> footTargetPosition;
                    if (this->robot->robotConfig["is_sim"]) {
                        // running in simulation
                        footSourcePosition = phaseSwitchFootGlobalPos.col(legId);
                        footTargetPosition = footHoldInWorldFrame.col(legId);
                        footTargetPosition[2] = footSourcePosition[2] + footholdPlanner->desiredFootholdsOffset(2, legId);                      
                    } else {
                        // swing in base frame
                        footSourcePosition = phaseSwitchFootLocalPos.col(legId);
                        footTargetPosition = footSourcePosition + constOffset;
                        if (legId<=1) {
                            footTargetPosition[0] = 0.30f;
                            
                        }
                        else {
                            footTargetPosition[0] = -0.17f;
                        }
                        footTargetPosition[1] = -0.145 * pow(-1, legId);
                        footTargetPosition[2] = -0.32f;
                    }                        
                    
                    SplineInfo splineInfo;
                    splineInfo.splineType = "BSpline";
                    swingFootTrajectories[legId] = SwingFootTrajectory(splineInfo, footSourcePosition, footTargetPosition, 1.f, 0.15);
                    cout << "[SwingLegController::Update leg " << legId << "  update footHoldInWorldFrame: \n"
                        << footHoldInWorldFrame.col(legId) << endl;
                }
            }
        } break;
        default : {
            for (int legId = 0; legId < 4; ++legId) {
                if (newLegState(legId) == LegState::SWING && newLegState(legId) != gaitGenerator->lastLegState(legId)) {
                    phaseSwitchFootLocalPos.col(legId) = robot->GetFootPositionsInBaseFrame().col(legId);
                }
            }
        } break;
    }
}

std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> QuadrupedRobot::qrSwingLegController::GetAction()
{
    std::cout << "qrSwingLegController GetAction() function" << std::endl;
    
}