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

#include "controllers/mpc/qr_mpc_stance_leg_controller.h"

using robotics::math::clip;

namespace Quadruped {

MPCStanceLegController::MPCStanceLegController(
    qrRobot *robot,
    qrGaitGenerator *gaitGenerator,
    qrStateEstimatorContainer *stateEstimators,
    qrComAdjuster *comAdjuster,
    qrPosePlanner *posePlanner,
    qrFootholdPlanner *footholdPlanner,
    qrUserParameters &userParameters,
    std::string configFilepath):

    TorqueStanceLegController(robot, gaitGenerator, stateEstimators, comAdjuster, posePlanner, footholdPlanner, userParameters, configFilepath),
    horizonLength(5),
    dtMPC(0.06),
    dt(0.002)
{
    /* A gait period consists of numHorizonL times MPC period.
     * Interval of two MPC updates is dtMPC. If MPC length is horizonLength，then this MPC considers dtMPC*horizonLength time.
     * Generally, the gait period T = dtMPC * HorizonLength * numHorizonL.
     */
    numHorizonL = std::max(2, int(gaitGenerator->fullCyclePeriod[0] / 0.4));
    iterationsInaMPC = round(dtMPC / dt);
    defaultIterationsInMpc = iterationsInaMPC;

    printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f, horizonLen: %d\n", dt,
           iterationsInaMPC, dtMPC, horizonLength);// 0.002, 15, 0.03

    useWBC = userParameters.useWBC;

    Reset(0);

    std::vector<float> QIN = param["stance_leg_params"][controlModeStr]["Q"].as<std::vector<float>>();
    for (u8 i(0); i < 12; ++i) {
        Q[i] = QIN[i];
    }
    printf("[MPC] Init: success!\n");
}


void MPCStanceLegController::Reset(float t)
{
    rpyComp.setZero();
    f_ff.setZero();
    f.setZero();

    mpcTable.resize(horizonLength, 4);
    mpcTable.setOnes();

    posDesiredinWorld = robot->basePosition;
    bodyHeight = robot->basePosition[2];
    yawDesTrue = robot->baseRollPitchYaw[2];
    yawTurnRate = 0;

    double maxForce = robot->totalMass * 9.81;
    float *weights = Q;
    float alpha = 4e-6;

    Vec3<float> inertia;
    inertia << robot->totalInertia(0,0), robot->totalInertia(1,1), robot->totalInertia(2,2);

    SetupProblem(dtMPC, horizonLength, 0.45, maxForce, robot->totalMass, inertia.data(), weights, alpha);

    iterationCounter = 0;

    mpcUpdated = false;

    if (useWBC) {
        auto& wbcData = robot->stateDataFlow.wbcData;
        for (int i=0; i< NumLeg; ++i) {
            wbcData.Fr_des[i] << 0, 0, float(maxForce/4);
        }

        /* In one iteration, MPC or WBC is conducted to ensure the frequency. */
        wbcData.allowAfterMPC = !mpcUpdated;

        wbcData.pBody_des[0] = posDesiredinWorld[0];
        wbcData.pBody_des[1] = posDesiredinWorld[1];
        wbcData.pBody_des[2] = robot->bodyHeight;

        wbcData.vBody_des[0] = 0;
        wbcData.vBody_des[1] = 0;
        wbcData.vBody_des[2] = 0.;

        wbcData.aBody_des.setZero();

        wbcData.pBody_RPY_des[0] = 0;
        wbcData.pBody_RPY_des[1] = 0;
        wbcData.pBody_RPY_des[2] = yawDesTrue;

        wbcData.vBody_Ori_des[0] = 0.;
        wbcData.vBody_Ori_des[1] = 0.;
        wbcData.vBody_Ori_des[2] = 0;

        wbcData.contact_state.setOnes();
    }
    std::cout << "[MPC] Reset: iterationCounter = " << iterationCounter << std::endl;
}


std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> MPCStanceLegController::GetAction()
{
    std::map<int, qrMotorCommand> legCommand;

    /* Run the MPC and the result will be stored in member %f_ff. */
    Run(legCommand, 0, 0);

    std::map<int, float> motorTorques;

    /* Map exerting force to joint torque using Tor = J^T * F. */
    for (int legId = 0; legId < NumLeg; ++legId) {
        motorTorques = this->robot->MapContactForceToJointTorques(legId, f_ff.col(legId));

        for (std::map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
            if (gaitGenerator->legState[legId] == LegState::EARLY_CONTACT && it->first % 3 == 0) {
                // MotorCommand temp{0., 0., 0., 1.0, it->second};
                qrMotorCommand temp{0., 100.0, 0., 3.0, it->second};
                legCommand[it->first] = temp;
            } else {
                // MotorCommand temp{0., 0., 0., 1.0, it->second};
                qrMotorCommand temp{0., 0., 0., 3.0, it->second};
                legCommand[it->first] = temp;
            }
        }
    }

    return {legCommand, f_ff};
}

void MPCStanceLegController::SetupCommand()
{
    /* Get robot current and desired pose and twist. */
    UpdateDesCommand();

    bodyHeight = desiredStateCommand->stateDes(2);

    float x_vel_cmd, y_vel_cmd, yaw_vel_cmd;

    x_vel_cmd = desiredStateCommand->stateDes(6);// in base frame
    y_vel_cmd = desiredStateCommand->stateDes(7);
    yaw_vel_cmd = desiredStateCommand->stateDes(11);

    /* A linear filter for linear and angular velocity commands.
     * And also clip the commands.
     */
    float x_filter(0.01f), y_filter(0.005f), yaw_filter(0.03f);
    xVelDes = xVelDes * (1 - x_filter) + x_vel_cmd * x_filter;//一阶低通数字滤波
    yVelDes = yVelDes * (1 - y_filter) + y_vel_cmd * y_filter;
    yawTurnRate = yawTurnRate * (1 - yaw_filter) + yaw_vel_cmd * yaw_filter;
    xVelDes = clip(xVelDes, -1.0f, 2.0f);
    yVelDes = clip(yVelDes, -0.6f, 0.6f);

    float yawCurrent = robot->GetBaseRollPitchYaw()[2];

    /* Plan desired yaw based on last palnned desired yaw angle. */
    yawDesTrue = yawDesTrue + dt * yawTurnRate;

    /* Normalize the desired yaw into -M_PI to M_PI. */
    if (yawDesTrue >= M_PI) {
        yawDesTrue -= M_2PI;
    } else if (yawDesTrue <= -M_PI) {
        yawDesTrue += M_2PI;
    }

    /* Critial point, dtheta/dt = R^T * w. If w > 0, theta_{t+1} > theta_{t},
     * which means when theta_{t} in (pi/2, pi], theta_{t+1} should be large then pi,
     * rather than in range (-pi, pi]. */
    if (yawCurrent > M_PI_2 && yawDesTrue < 0) {
        yawDesTrue += M_2PI;
    } else if (yawCurrent < -M_PI_2 && yawDesTrue > 0) {
        yawDesTrue -= M_2PI;
    }

    rollDes = desiredStateCommand->stateDes(3);
    pitchDes = desiredStateCommand->stateDes(4);
}


void MPCStanceLegController::Run(std::map<int, qrMotorCommand> &legCommand, int gaitType, int robotMode)
{

    SetupCommand();

    Vec4<bool> contactState = contacts;
    auto &seResult = robot->stateDataFlow;
    Vec3<float> vBodyInBaseFrame = robot->GetBaseVelocityInBaseFrame();
    Vec3<float> rpy = robot->GetBaseRollPitchYaw();


    Vec3<float> vDesRobot(xVelDes, yVelDes, 0); /* Desired linear velocity in body frame. */
    vDesWorld = seResult.baseRMat * vDesRobot; /* Desired linear velocity in world frame. */
    Vec3<float> v_robot = seResult.baseVInWorldFrame; /* Actual linear velocity in world frame. */
    pFoot = robot->GetFootPositionsInWorldFrame(); /* Actual foot positions in world frame. */

    /* Desired position in world is accumulated from acceleration. */
    Vec3<float> desiredP = desiredStateCommand->stateDes.head(3);
    posDesiredinWorld += dt * Vec3<float>(vDesWorld[0], vDesWorld[1], 0);
    posDesiredinWorld[2] = 0.99 * (bodyHeight + (bodyHeight - robot->basePosition[2])) + 0.01 * posDesiredinWorld[2];

    rpyComp[1] = pitchDes;

    /* Add height compensation according to the num of swinging legs
     * and pitch compensation when the robot is walking back.
     * These compensations are quite intuational. */
    for (int legId(0); legId < NumLeg; ++legId) {
        if (gaitGenerator->desiredLegState[legId] == LegState::SWING) {
            bodyHeight += 0.02 * std::sin(gaitGenerator->normalizedPhase[legId] * M_PI);// height compensation
            if (desiredStateCommand->stateDes(6, 0) < -0.01) {
                rpyComp[1] = rpyComp[1] - 0.1 * std::sin(gaitGenerator->normalizedPhase[legId] * M_PI);// pitch compensation
            }
            break;
        }
    }

    /* Get desired CoM destination according to the average of the desired foot positions. */
    Eigen::Matrix<float, 3, 4> desiredFootholdsInWorldFrame = desiredStateCommand->footTargetPositionsInWorldFrame;
    Vec3<float> comDestination(0, 0, 0);
    for (int i(0); i < NumLeg; ++i) {
        if (!contactState[i]) {
            comDestination += desiredFootholdsInWorldFrame.col(i);
        } else {
            comDestination += pFoot.col(i);
        }
    }
    comDestination /= 4.f;

    float st = 0;
    float et = gaitGenerator->fullCyclePeriod[0];
    robotics::math::qrSpline::Point res;
    float t = 1;
    float dutyF = gaitGenerator->dutyFactor[0];
    if (gaitGenerator->desiredLegState[0] == LegState::SWING) {
        t = gaitGenerator->phaseInFullCycle[0] - dutyF;
    } else if (gaitGenerator->desiredLegState[1] == LegState::SWING) {
        t = gaitGenerator->phaseInFullCycle[1] - dutyF;
    } else if (gaitGenerator->phaseInFullCycle[0] < gaitGenerator->phaseInFullCycle[1]) {// 0-leg just finished swinging
        t = gaitGenerator->phaseInFullCycle[0] + (1 - dutyF);
    } else {
        t = gaitGenerator->phaseInFullCycle[1] + (1 - dutyF);
    }
    t *= 2.0f;

    /* Update desired position and linear velocity in world frame by Spline. */
    Vec12<float> baseStartState = footholdPlanner->firstSwingBaseState;
    for (u8 axis(0); axis < 2; ++axis) {
        auto sPoint = robotics::math::qrSpline::Point(baseStartState[axis], baseStartState[3 + axis], 0.05);
        auto ePoint = robotics::math::qrSpline::Point(comDestination[axis], vDesWorld[axis], 0.05);
        res.x = (1 - t) * sPoint.x + t * ePoint.x;
        res.xd = ePoint.xd;
        posDesiredinWorld[axis] = res.x;
        vDesWorld[axis] = res.xd;
    }

    /* Update MPC table. */
    Vec4<float> progress = gaitGenerator->phaseInFullCycle;
    float dPhase = 1.0 / (numHorizonL * horizonLength);
    for (int i = 0; i < horizonLength; i++) {
        for (int j = 0; j < NumLeg; ++j) {
            float ithMPCPhase = progress[j] + i * dPhase;
            while (ithMPCPhase > 1.0) {
                ithMPCPhase -= 1.0;
            }

            /* MPC only considers the stance leg. */
            if (ithMPCPhase < gaitGenerator->dutyFactor[j] || gaitGenerator->legState[j] == LegState::EARLY_CONTACT) {
                mpcTable(i, j) = 1;
            } else {
                mpcTable(i, j) = 0;
            }
        }
    }

    for (int legId = 0; legId < NumLeg; ++legId) {
        mpcTable(0, legId) = float(contactState[legId]);
    }

    UpdateMPC(robot);

    if (useWBC) {

        /* In one iteration, MPC or WBC is conducted to ensure the time per iteration. */
        seResult.wbcData.allowAfterMPC = !mpcUpdated;

        Vec3<float> offsetP = seResult.baseRMat * Vec3<float>(0.018f, 0, 0);
        seResult.wbcData.pBody_des[0] = posDesiredinWorld[0] + offsetP[0];
        seResult.wbcData.pBody_des[1] = posDesiredinWorld[1] + offsetP[1];
        seResult.wbcData.pBody_des[2] = bodyHeight;

        seResult.wbcData.vBody_des[0] = vDesWorld[0];
        seResult.wbcData.vBody_des[1] = vDesWorld[1];
        seResult.wbcData.vBody_des[2] = 0.;

        seResult.wbcData.aBody_des.setZero();

        seResult.wbcData.pBody_RPY_des[0] = rpyComp[0];
        seResult.wbcData.pBody_RPY_des[1] = rpyComp[1];
        seResult.wbcData.pBody_RPY_des[2] = yawDesTrue;

        seResult.wbcData.vBody_Ori_des[0] = 0.;
        seResult.wbcData.vBody_Ori_des[1] = 0.;
        seResult.wbcData.vBody_Ori_des[2] = yawTurnRate;

        seResult.wbcData.contact_state = contactState;
    }
    ++iterationCounter;
}


void MPCStanceLegController::UpdateMPC(qrRobot *robot)
{
    /* To ensure the frequency, MPC is calculated twice in an MPC horizon.
     * MPC is calculated at every first 50 iterations.
     * We found that the effect is as same as that of calculating MPC every iteration. */
    if (iterationCounter % (iterationsInaMPC / 2) == 0 || iterationCounter < 50) {

        /* Set limitation to the desired pose position in world frame. */
        Vec3<float> p = robot->GetBasePosition();

        const float max_posx_error = 0.1f;
        const float max_posy_error = 0.1f;
        float xStart = posDesiredinWorld[0];
        float yStart = posDesiredinWorld[1];

        xStart = clip(xStart, p[0] - max_posx_error, p[0] + max_posx_error);
        yStart = clip(yStart, p[1] - max_posy_error, p[1] + max_posy_error);

        posDesiredinWorld[0] = xStart;
        posDesiredinWorld[1] = yStart;

        Vec3<float> omega_des(0, 0, yawTurnRate);

        /* Initial trajectory: desired roll pitch yaw, pose, angular velosity and linear velocity. */
        float trajInitial[12] = {rpyComp[0],   rpyComp[1],   yawDesTrue,
                                 xStart,       yStart,       bodyHeight,
                                 omega_des[0], omega_des[1], omega_des[2],
                                 vDesWorld[0], vDesWorld[1], 0.f};

        /* Predict the future state by accmulating the velocity. */
        for (int i = 0; i < horizonLength; ++i) {
            for (int j = 0; j < 12; ++j) trajAll[12 * i + j] = trajInitial[j];
            if (i == 0) {
                trajAll[2] = yawDesTrue;
            } else {
                trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * yawTurnRate;
                trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * vDesWorld[0];
                trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * vDesWorld[1];
            }
        }
        SolveDenseMPC(robot);
        mpcUpdated = true;
    } else {
      mpcUpdated = false;
    }
}


void MPCStanceLegController::SolveDenseMPC(qrRobot *robot)
{
    auto &seResult = robot->stateDataFlow;

    /* Get base/CoM information. */
    Vec3<float> rpy  = robot->GetBaseRollPitchYaw();
    Vec3<float> pos  = robot->GetBasePosition();
    Quat<float> quat = robot->GetBaseOrientation();
    Eigen::Matrix<float, 3, 4> footPosInBaseFrame = robot->GetFootPositionsInBaseFrame();

    /* Get foothold position to the CoM. */
    Eigen::Matrix<float, 3, 4> foot2ComInWorldFrame = seResult.baseRMat * (footPosInBaseFrame.colwise() - robot->comOffset);
    float *r = foot2ComInWorldFrame.data();

    SolveMPCKernel(pos, seResult.baseVInWorldFrame, quat, seResult.baseWInWorldFrame, foot2ComInWorldFrame, rpy, trajAll, mpcTable.data());

    /* Transform from reacting force to acting force of motors. */
    for (int leg = 0; leg < NumLeg; ++leg) {
        for (int axis = 0; axis < 3; ++axis) {
            f(axis, leg) = GetMPCSolution(leg * 3 + axis);
        }
        f_ff.col(leg) = -seResult.baseRMat.transpose() * f.col(leg);

        seResult.wbcData.Fr_des[leg] = f.col(leg);
    }
}

} // namespace Quadruped
