#include "controllers/mpc/mit_mpc_stance_leg_controller.h"
#include "controllers/mpc/qp_torque_optimizer_mpc.h"

using robotics::math::clip;

namespace Quadruped {

    /**
 * @brief 参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive
 *   Control 一个步态周期由 numHorizonL 个 mpc 周期组成, 电机控制频率按 (1/dt)Hz 处理
 *   mpc 一次迭代间隔dtMPC, 长度为horizonLength，则这次MPC 考虑了未来dtMPC*horizonLength 的时间.
 */
    MITConvexMPCStanceLegController::MITConvexMPCStanceLegController(Robot *robot,
                                                                     GaitGenerator *gaitGenerator,
                                                                     StateEstimatorContainer<float> *stateEstimators,
                                                                     ComAdjuster *comAdjuster,
                                                                     PosePlanner *posePlanner,
                                                                     FootholdPlanner *footholdPlanner,
                                                                     UserParameters &userParameters,
                                                                     std::string configFilepath)
        : TorqueStanceLegController(robot, gaitGenerator, stateEstimators, comAdjuster,
                                    posePlanner, footholdPlanner, userParameters, configFilepath),
          horizonLength(5),
          dtMPC(0.06),
          dt(0.002)
    {
        numHorizonL = std::max(2, int(gaitGenerator->fullCyclePeriod[0] / 0.4));// 1.0s/ 0.4s = 2
        iterationsInaMPC = round(dtMPC / dt);                                   // (close loop)控制频率 1s/0.002s = 500Hz,  60/2 = 30 times
        defaultIterationsInMpc = iterationsInaMPC;
        printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f, horizonLen: %d\n", dt,
               iterationsInaMPC, dtMPC, horizonLength);// 0.002, 15, 0.03
        useWBC = userParameters.useWBC;
        Reset(0);
        std::cout << "[MPC Init]: controlModeStr = " << controlModeStr << param["stance_leg_params"][controlModeStr]["Q"] << std::endl;
        std::vector<float> QIN = param["stance_leg_params"][controlModeStr]["Q"].as<std::vector<float>>();
        for (u8 i(0); i < 12; ++i) {
            Q[i] = QIN[i];
        }
        std::cout << "[MPC Init]: success!" << std::endl;
    }

    void MITConvexMPCStanceLegController::Reset(float t)
    {
        // TorqueStanceLegController::Reset(currentTime);
        rpy_comp.setZero();
        rpy_int.setZero();
        f_ff.setZero();
        f.setZero();
        _mpcTable.resize(horizonLength, 4);
        _mpcTable.setOnes();
        posDesiredinWorld = robot->basePosition;
        _body_height = robot->basePosition[2];
        _yaw_des_true = robot->baseRollPitchYaw[2];
        _yaw_turn_rate = 0;

        double maxForce = robot->totalMass * 9.81;
        float *weights = Q;
        float alpha = 4e-6;
        Vec3<float> inertia;// todo
        // inertia << .07f, 0.26f, 0.242f; // mini cheetah
        inertia << 0.24f, 0.8f, 1.0f;// a1
        // inertia << 0.4f, 1.15f, 1.0f; // aliengo + z1 arm
        SetupProblem(dtMPC, horizonLength, 0.45, maxForce, robot->totalMass, inertia.data(), weights, alpha);// 0.4
        iterationCounter = 0;
        std::cout << "[MPC Reset]: iterationCounter = " << iterationCounter << std::endl;
    }

    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> MITConvexMPCStanceLegController::GetAction()
    {
        std::map<int, MotorCommand> legCommand;
        // MITTimer tik;
        Run(legCommand, 0, 0);
        // printf("MPC time SOLVE TIME: %.3f\n", tik.getMs());

        std::map<int, float> motorTorques;
        for (int legId = 0; legId < NumLeg; ++legId) {
            motorTorques = this->robot->MapContactForceToJointTorques(legId, f_ff.col(legId));
            for (std::map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                if (gaitGenerator->legState[legId] == LegState::EARLY_CONTACT && it->first % 3 == 0) {
                    MotorCommand temp{0., 50, 0., 3., it->second};
                    // MotorCommand temp{0., 0., 0., 1., it->second};
                    legCommand[it->first] = temp;
                } else {
                    MotorCommand temp{0., 0., 0., 2., it->second};
                    legCommand[it->first] = temp;
                }
            }
        }

        return {legCommand, f_ff};
    }

    void MITConvexMPCStanceLegController::_SetupCommand()
    {
        UpdateDesCommand();

        _body_height = desiredStateCommand->stateDes(2);
        float x_vel_cmd, y_vel_cmd, yaw_vel_cmd;
        // 手柄数据先暂时设置为默认值，按 A 键 开启手柄控制--旋转角速度和x,y方向上的线速度
        x_vel_cmd = desiredStateCommand->stateDes(6);// in base frame
        y_vel_cmd = desiredStateCommand->stateDes(7);
        yaw_vel_cmd = desiredStateCommand->stateDes(11);

        float x_filter(0.01), y_filter(0.005), yaw_filter(0.03);
        _x_vel_des = _x_vel_des * (1 - x_filter) + x_vel_cmd * x_filter;//一阶低通数字滤波
        _y_vel_des = _y_vel_des * (1 - y_filter) + y_vel_cmd * y_filter;
        _yaw_turn_rate = _yaw_turn_rate * (1 - yaw_filter) + yaw_vel_cmd * yaw_filter;
        _x_vel_des = clip(_x_vel_des, -1.0f, 2.0f);
        _y_vel_des = clip(_y_vel_des, -0.6f, 0.6f);

        float yaw_ = robot->GetBaseRollPitchYaw()[2];
        // plan desired yaw based on last palnned desired yaw angle.
        _yaw_des_true = _yaw_des_true + dt * _yaw_turn_rate;
        // normalization
        if (_yaw_des_true >= M_PI) {
            _yaw_des_true -= M_2PI;
        } else if (_yaw_des_true <= -M_PI) {
            _yaw_des_true += M_2PI;
        }
        // critial point, dtheta/dt = R^T * w. If w > 0, theta_{t+1} > theta_{t}.
        // which means when theta_{t} in (pi/2, pi], theta_{t+1} should be large then pi,
        // rather than in range (-pi, pi].
        if (yaw_ > M_PI_2 && _yaw_des_true < 0) {
            _yaw_des_true += M_2PI;
        } else if (yaw_ < -M_PI_2 && _yaw_des_true > 0) {
            _yaw_des_true -= M_2PI;
        }
        _roll_des = desiredStateCommand->stateDes(3);
        _pitch_des = desiredStateCommand->stateDes(4);
    }

    void MITConvexMPCStanceLegController::Run(std::map<int, MotorCommand> &legCommand, int gaitType, int robotMode)
    {
        _SetupCommand();

        Vec4<bool> contactState = contacts;
        auto &seResult = robot->stateDataFlow;// 状态估计
        Vec3<float> vBodyInBaseFrame = robot->GetEstimatedVelocityInBaseFrame();
        Vec3<float> rpy = robot->GetBaseRollPitchYaw();
        Vec3<float> vDesRobot(_x_vel_des, _y_vel_des, 0);// 身体坐标系下的期望线速度
        vDesWorld = seResult.baseRMat * vDesRobot;       //世界坐标系下的期望线速度
        Vec3<float> v_robot = seResult.baseVInWorldFrame;  //世界坐标系下的机器人实际速度
        pFoot = robot->GetFootPositionsInWorldFrame();

        Vec3<float> error;
        //非站立下的期望位置，通过累加期望速度完成. only for ground.
        Vec3<float> desiredP = desiredStateCommand->stateDes.head(3);
        posDesiredinWorld += dt * Vec3<float>(vDesWorld[0], vDesWorld[1], 0);
        posDesiredinWorld[2] = 0.99 * (_body_height + (_body_height - robot->basePosition[2])) + 0.01 * posDesiredinWorld[2];

        rpy_comp[1] = _pitch_des;
        for (int legId(0); legId < NumLeg; ++legId) {
            if (gaitGenerator->desiredLegState[legId] == LegState::SWING) {
                _body_height += 0.02 * std::sin(gaitGenerator->normalizedPhase[legId] * M_PI);// height compensation
                if (desiredStateCommand->stateDes(6, 0) < -0.01) {
                    rpy_comp[1] = rpy_comp[1] - 0.1 * std::sin(gaitGenerator->normalizedPhase[legId] * M_PI);// pitch compensation
                }
                break;
            }
        }

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
        robotics::math::Spline::Point res;
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
        Vec12<float> baseStartState = footholdPlanner->firstSwingBaseState;

        for (u8 axis(0); axis < 2; ++axis) {
            auto sPoint = robotics::math::Spline::Point(baseStartState[axis], baseStartState[3 + axis], 0.05);
            auto ePoint = robotics::math::Spline::Point(comDestination[axis], vDesWorld[axis], 0.05);
            res.x = (1 - t) * sPoint.x + t * ePoint.x;
            res.xd = ePoint.xd;
            posDesiredinWorld[axis] = res.x;
            vDesWorld[axis] = res.xd;
        }

        // update mpc_table
        Vec4<float> progress = gaitGenerator->phaseInFullCycle;
        float dPhase = 1.0 / (numHorizonL * horizonLength);
        for (int i = 0; i < horizonLength; i++) {
            for (int j = 0; j < NumLeg; ++j) {
                float ithMPCPhase = progress[j] + i * dPhase;
                while (ithMPCPhase > 1.0) {
                    ithMPCPhase -= 1.0;
                }
                if (ithMPCPhase < gaitGenerator->dutyFactor[j] || gaitGenerator->legState[j] == LegState::EARLY_CONTACT) {
                    _mpcTable(i, j) = 1;// this leg is in stance, mpc should consider this leg.
                } else {
                    _mpcTable(i, j) = 0;
                }
            }
        }
        for (int legId = 0; legId < 4; ++legId) {
            _mpcTable(0, legId) = float(contactState[legId]);
        }
        _UpdateMPC(robot);

        // Update For WBC
        if (useWBC) {
            seResult.wbcData.allowAfterMPC = !mpcUpdated;
            seResult.wbcData.pBody_des[0] = robot->basePosition[0];//posDesiredinWorld[0];
            seResult.wbcData.pBody_des[1] = robot->basePosition[1];//posDesiredinWorld[1];
            seResult.wbcData.pBody_des[2] = _body_height;

            seResult.wbcData.vBody_des[0] = vDesWorld[0];
            seResult.wbcData.vBody_des[1] = vDesWorld[1];
            seResult.wbcData.vBody_des[2] = 0.;
            // seResult.wbcData.vBody_des[2] = vDesWorld[2];

            seResult.wbcData.aBody_des.setZero();

            seResult.wbcData.pBody_RPY_des[0] = rpy_comp[0];
            seResult.wbcData.pBody_RPY_des[1] = rpy_comp[1];
            seResult.wbcData.pBody_RPY_des[2] = _yaw_des_true;

            seResult.wbcData.vBody_Ori_des[0] = 0.;
            seResult.wbcData.vBody_Ori_des[1] = 0.;
            seResult.wbcData.vBody_Ori_des[2] = _yaw_turn_rate;

            seResult.wbcData.contact_state = contactState;
        }
        ++iterationCounter;
    }

    void MITConvexMPCStanceLegController::_UpdateMPC(Robot *_quadruped)
    {
        if (iterationCounter % (iterationsInaMPC / 2) == 0 || iterationCounter < 50) {
            // if (iterationCounter % 2 == 0) {

            Vec3<float> p = _quadruped->GetBasePosition();
            const float max_posx_error = 0.1f;
            const float max_posy_error = 0.1f;
            float xStart = posDesiredinWorld[0];
            float yStart = posDesiredinWorld[1];
            xStart = clip(xStart, p[0] - max_posx_error, p[0] + max_posx_error);
            yStart = clip(yStart, p[1] - max_posy_error, p[1] + max_posy_error);
            posDesiredinWorld[0] = xStart;
            posDesiredinWorld[1] = yStart;

            Vec3<float> omega_des(0, 0, _yaw_turn_rate);
            // omega_des = _quadruped->stateDataFlow.baseRMat * omega_des; // base -> world // todo

            float trajInitial[12] = {rpy_comp[0],  // 0
                                     rpy_comp[1],  // 1
                                     _yaw_des_true,// 2
                                     // yawStart,    // 2
                                     xStart,      // 3
                                     yStart,      // 4
                                     _body_height,// 5
                                                  //  posDesiredinWorld[2],
                                     omega_des[0],// 6
                                     omega_des[1],// 7
                                     omega_des[2],//_yaw_turn_rate,     // 8
                                     vDesWorld[0],// 9
                                     vDesWorld[1],// 10
                                     0.f};        // 11
                                                  //  vDesWorld[2]};

            for (int i = 0; i < horizonLength; ++i) {
                for (int j = 0; j < 12; ++j) trajAll[12 * i + j] = trajInitial[j];

                if (i == 0)// start at current position,  TODO consider not doing this
                {
                    trajAll[2] = _yaw_des_true;
                } else {
                    trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * vDesWorld[0];// _x_vel_des;
                    trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * vDesWorld[1];// _y_vel_des;
                    trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
                }
            }
            // MITTimer solveTimer;
            _SolveDenseMPC(_quadruped);
            // printf("MPC SOLVE TIME: %.3f\n", solveTimer.getMs());
            mpcUpdated = true;
        } else {
            mpcUpdated = false;
        }
    }

    void MITConvexMPCStanceLegController::_SolveDenseMPC(Robot *_quadruped)
    {
        auto &seResult = _quadruped->stateDataFlow;

        Vec3<float> rpy = _quadruped->GetBaseRollPitchYaw();
        Vec3<float> pos = _quadruped->GetBasePosition();
        Quat<float> quat = _quadruped->GetBaseOrientation();
        Eigen::Matrix<float, 3, 4> footPosInBaseFrame = _quadruped->GetFootPositionsInBaseFrame();

        float *p = pos.data();
        float *v = seResult.baseVInWorldFrame.data();
        float *w = seResult.baseWInWorldFrame.data();
        float *q = quat.data();

        Eigen::Matrix<float, 3, 4> foot2ComInWorldFrame = seResult.baseRMat * (footPosInBaseFrame.colwise() - _quadruped->comOffset);
        float *r = foot2ComInWorldFrame.data();// colMajor

        // MITTimer t2;
        SolveMPCKernel(p, v, q, w, r, rpy.data(), trajAll, _mpcTable.data());
        // printf("SolveDenseMPC time %f ms\n", t2.getMs());

        for (int leg = 0; leg < NumLeg; ++leg) {
            for (int axis = 0; axis < 3; ++axis) {
                f(axis, leg) = GetMPCSolution(leg * 3 + axis);
            }
            f_ff.col(leg) = -seResult.baseRMat.transpose() * f.col(leg);
            seResult.wbcData.Fr_des[leg] = f.col(leg);
        }
    }

}// namespace Quadruped
