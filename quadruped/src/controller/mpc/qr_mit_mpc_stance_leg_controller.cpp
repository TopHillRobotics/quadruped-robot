#include "controller/mpc/qr_mit_mpc_stance_leg_controller.h"

#define M_2PI 6.28318530718 // 2 * PI

extern std::unordered_map<int, std::string> modeMap;

// 参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive
// Control 一个步态周期由 numHorizonL 个 mpc 周期组成, 电机控制频率按 (1/dt)Hz 处理
// mpc 一次迭代间隔dtMPC, 长度为horizonLength，则这次MPC 考虑了未来dtMPC*horizonLength 的时间.
qrMITConvexMPCStanceLegController::qrMITConvexMPCStanceLegController(
    qrRobot *robot,
    qrGaitGenerator *gaitGenerator,
    qrRobotEstimator *robotVelocityEstimator,
    qrGroundSurfaceEstimator *groundEstimatorIn,
    qrComPlanner *comPlanner,
    qrFootholdPlanner *footholdPlanner,
    Eigen::Matrix<float, 3, 1> desired_speed,
    float desiredTwistingSpeed,
    float desiredBodyHeight,
    int numLegs,
    std::string configFilepath,
    std::vector<float> frictionCoeffs)
    : qrStanceLegController(robot, gaitGenerator, robotVelocityEstimator, groundEstimatorIn, comPlanner,
       footholdPlanner, desired_speed, desiredTwistingSpeed, desiredBodyHeight, numLegs, configFilepath, frictionCoeffs),
      horizonLength(5), // 5
      dtMPC(0.06), // 0.02 0.06
      dt(0.002)
{

    // dtMPC = gaitGenerator->fullCyclePeriod[0] / (3*horizonLength); // one mpc update, will consider next dtMPC time; 0.6 / 10 = 60ms
    numHorizonL = std::max(2, int(gaitGenerator->fullCyclePeriod[0] / 0.4)); // 1.0s/ 0.4s = 2
    // numHorizonL = 3; //gaitGenerator->fullCyclePeriod[0] / 0.4)
    iterationsInaMPC = round(dtMPC / dt); // (close loop)控制频率 1s/0.002s = 500Hz,  60/2 = 30 times
    default_iterations_in_mpc = iterationsInaMPC;
    printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f, horizonLen: %d\n", dt,
           iterationsInaMPC, dtMPC, horizonLength);// 0.002, 15, 0.03
    // setup_problem(dtMPC, horizonLength, 0.4, 120);
    //useWBC = userParameters.useWBC;
    Reset(0);
    std::cout << "init mit mpc success!" <<std::endl;

    // std::vector<float> v = param["stance_leg_params"][controlModeStr]["X_weight"].as<std::vector<float>>();
    // this->XWeight = Eigen::MatrixXf::Map(&v[0], 13, 1);
    // std::cout << "Xweight = " <<XWeight << std::endl;
    //std::cout << "controlModeStr " << controlModeStr <<  param["stance_leg_params"][controlModeStr]["Q"] << std::endl;
    // if use advanced trot, use Q matrix
    std::cout << "MPC path:" << configFilepath << std::endl;
    YAML::Node param = YAML::LoadFile(configFilepath);
    std::vector<float> QIN =
        param["stance_leg_params"][modeMap[LocomotionMode::ADVANCED_TROT_LOCOMOTION]]["Q"].as<std::vector<float>>();
    // float Q[12] = {2.5, 2.5, 2.5, 30, 30, 50, 0.1, 0.1, 0.5, 0.1, 0.1, 0.1};
    // float Q[12] = {2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1}; // origin
    // float Q[12] = {3, 3, 5, 40, 40, 50, 0., 0., 0.5, 5, 5, 1};
    // float Q[12] = {10, 10, 5, 40, 60, 100, 0., 0, 0.5, 5, 5, 1};

    for(u8 i(0); i<12; ++i) {
        Q[i] = QIN[i];
    }
    initStateDes();
}

void qrMITConvexMPCStanceLegController::Reset(float t)
{
    // TorqueStanceLegController::Reset(currentTime);

    initSparseMPC();
    rpy_comp.setZero();
    rpy_int.setZero();

    f_ff.setZero();
    f.setZero();
    _mpcTable.resize(horizonLength,4);
    _mpcTable.setOnes();
    world_position_desired = robot->GetBasePosition();
    _body_height = robot->GetBasePosition()[2];
    _yaw_des_true = robot->GetBaseRollPitchYaw()[2];
    _yaw_turn_rate = 0;
    for (int i = 0; i < 4; i++) firstSwing[i] = true;
    firstRun = true;
    iterationCounter = 0;

    // TODO: whether to consider the mass of legs
    double maxForce = robot->GetBodyMass() * 9.81; // 150, 300
    setup_problem(dtMPC, horizonLength, 0.45, maxForce, robot->GetBodyMass()); // 0.4

    int jcqp_max_iter = 10000;
    double jcqp_rho = 0.0000001;
    double jcqp_sigma = 0.00000001;
    double jcqp_alpha = 1.5;
    double jcqp_terminate = 0.1;
    double use_jcqp = 0.0;

    update_solver_settings(jcqp_max_iter, jcqp_rho, jcqp_sigma, jcqp_alpha,
                           jcqp_terminate, use_jcqp);
    std::cout << "[mit Reset]: iterationCounter = " << iterationCounter << std::endl;
}

void qrMITConvexMPCStanceLegController::recompute_timing(int iterationsPerMpcStep)
{
    iterationsInaMPC = iterationsPerMpcStep;
    dtMPC = dt * iterationsPerMpcStep;
}

void qrMITConvexMPCStanceLegController::_SetupCommand()
{
    UpdateDesCommand();

    _body_height = stateDes(2);
    float x_vel_cmd, y_vel_cmd, yaw_vel_cmd;
    // 手柄数据先暂时设置为默认值，按 A 键 开启手柄控制--旋转角速度和x,y方向上的线速度
    // x_vel_cmd = desiredStateCommand->vDesInBodyFrame[0]; // in base frame
    // y_vel_cmd = desiredStateCommand->vDesInBodyFrame[1];
    // yaw_vel_cmd = desiredStateCommand->wDesInBodyFrame[2];
    x_vel_cmd = stateDes(6); // in base frame
    y_vel_cmd = stateDes(7);
    yaw_vel_cmd = stateDes(11);

    // _x_vel_des = x_vel_cmd;
    // _y_vel_des = y_vel_cmd;
    // _yaw_turn_rate = yaw_vel_cmd;
    float x_filter(0.01), y_filter(0.005), yaw_filter(0.03); // y(0.006)
    _x_vel_des = _x_vel_des * (1 - x_filter) + x_vel_cmd * x_filter;//一阶低通数字滤波
    _y_vel_des = _y_vel_des * (1 - y_filter) + y_vel_cmd * y_filter;
    _yaw_turn_rate = _yaw_turn_rate * (1 - yaw_filter) + yaw_vel_cmd * yaw_filter;

    if (_x_vel_des > 2.0) {
        _x_vel_des = 2.0;
    } else if (_x_vel_des < -1.0) {
        _x_vel_des = -1.0;
    }
    if (_y_vel_des > 0.6) {
        _y_vel_des = 0.6;
    } else if (_y_vel_des < -0.6) {
        _y_vel_des = -0.6;
    }
    float yaw_ = robot->GetBaseRollPitchYaw()[2];
    // plan desired yaw based on last palnned desired yaw angle.
    // _yaw_des = yaw_ + dt * _yaw_turn_rate;
    _yaw_des_true = _yaw_des_true + dt * _yaw_turn_rate; // 涉及到了状态估计中的欧拉角
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
    // std::cout << "theta " << yaw_ << ", " << _yaw_des_true << "\n ";
    _roll_des = stateDes(3);
    _pitch_des = stateDes(4);
}

std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrMITConvexMPCStanceLegController::GetAction()
{

    std::map<int, qrMotorCommand> legCommand;
    // MITTimer tik;
    run(legCommand, 0, 0);
    // printf("MPC time SOLVE TIME: %.3f\n", tik.getMs());

    std::map<int, float> motorTorques;
    for (int legId = 0; legId < NumLeg; ++legId) {
        motorTorques = this->robot->state.MapContactForceToJointTorques(legId, f_ff.col(legId));
        for (std::map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
            if (gaitGenerator->legState[legId]==LegState::EARLY_CONTACT && it->first%3==0) {
                qrMotorCommand temp{0., 50, 0., 3., it->second};
                // MotorCommand temp{0., 0., 0., 1., it->second};
                legCommand[it->first] = temp;
            } else {
                qrMotorCommand temp{0., 0., 0., 2., it->second};
                legCommand[it->first] = temp;
            }
        }
    }
    return {legCommand, f_ff};
}

void qrMITConvexMPCStanceLegController::run(std::map<int, qrMotorCommand>& legCommand, int gaitType, int robotMode)
{
    // std::cout << "iterationCounter = " << iterationCounter << std::endl;
    _SetupCommand();

    Vec4<bool> contactState = contacts;

    //auto &seResult = robot->stateDataFlow; // 状态估计
    Vec3<float> vBodyInBaseFrame = robot->GetBaseVelocity();
    // gait->setIterations(iterationsInaMPC, iterationCounter);//步态周期计算
    Vec3<float> rpy = robot->GetBaseRollPitchYaw();
    // integrate position setpoint
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0); // 身体坐标系下的期望线速度
    v_des_world = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * v_des_robot;  //世界坐标系下的期望线速度
    Vec3<float> v_robot = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * robot->GetBaseVelocity();    //世界坐标系下的机器人实际速度

    // Integral-esque pitche and roll compensation
    // 积分达到补偿值*******************************
    // 为了保持在运动过程中身体与地面平行
    // if (fabs(v_robot[0]) > .2)// avoid dividing by zero
    // {
    //     rpy_int[1] += dt * (_pitch_des - rpy[1]) / v_robot[0];
    // }
    // if (fabs(v_robot[1]) > 0.1) {
    //     rpy_int[0] += dt * (_roll_des - rpy[0]) / v_robot[1];
    // }
    // //初始角度限幅
    // rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);//-0.25~0.25
    // rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
    // rpy_comp[1] = v_robot[0] * rpy_int[1];// compensation 补偿值
    // rpy_comp[0] = v_robot[1] * rpy_int[0];// turn off for pronking

    float pitchFilter = 0.01; // 0.01
    // rpy_comp[0] = _roll_des;
    // rpy_int[1] = pitchFilter*_pitch_des + (1-pitchFilter)*rpy_int[1];
    rpy_comp[1] = _pitch_des;
    //得到世界坐标系下的足端位置. 机身坐标+机身旋转矩阵^T*（侧摆关节在机身下坐标+足底在侧摆关节下坐标）
    pFoot = robot->state.GetFootPositionsInWorldFrame();

    Vec3<float> error;
    //非站立下的期望位置，通过累加期望速度完成. only for ground.
    Vec3<float> desiredP = stateDes.head(3);
    // world_position_desired[0] = desiredP[0];
    // world_position_desired[1] = desiredP[1];
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    world_position_desired[2] = 0.99*(_body_height + (_body_height-robot->GetBasePosition()[2])) + 0.01 *world_position_desired[2];
    for (int legId(0); legId < NumLeg; ++legId) {
        if (gaitGenerator->desiredLegState[legId] == LegState::SWING) {
            _body_height += 0.02 * std::sin(gaitGenerator->normalizedPhase[legId]*M_PI); // height compensation
            if (stateDes(6,0) < -0.01) {
                rpy_comp[1] = rpy_comp[1] - 0.1 * std::sin(gaitGenerator->normalizedPhase[legId]*M_PI); // pitch compensation
            }
            break;
        }
    }
    // world_position_desired[2] = robot->basePosition[2];
    // world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], v_des_world[2]);
    /*
    // jump, 50Hz
    if (iterationCounter >=50) {
        if (iterationCounter < 550) {
            _body_height -= (iterationCounter-50)/500.0 * 0.10;
            v_des_world[2] = -0.2f;
        } else if (iterationCounter >= 550 && iterationCounter < 1000) {
            _body_height = _body_height - 0.1 + (iterationCounter-550)/450.0 * 0.05;
            v_des_world[2] = 0.11f;
        } else {
            _body_height = 0.32;
            v_des_world[2] = 0.f;
        }
    }
    */

//    Visualization2D& vis = seResult.visualizer;
//    Eigen::Matrix<float,3,4> desiredFootholdsInWorldFrame = desiredStateCommand->footTargetPositionsInWorldFrame;
//    Vec3<float> comDestination(0,0,0);
//    // comDestination = desiredFootholdsInWorldFrame.rowwise().mean();
//    // Vec3<float> meanPf = pFoot.rowwise().mean();

//    for (int i(0); i<4; ++i) {
//        if (!contactState[i]) {
//            comDestination += desiredFootholdsInWorldFrame.col(i);
//        } else {
//            comDestination += pFoot.col(i);
//        }
//    }
//    comDestination /= 4.f;
//    // std::cout << "pFoot = " << pFoot << std::endl;
//    // std::cout << "meanPf = " << meanPf << std::endl;
//    // std::cout << "drs = " << desiredFootholdsInWorldFrame <<std::endl;
//    // std::cout << "comDest = " << comDestination << std::endl;
//    float st = 0;
//    float et = gaitGenerator->fullCyclePeriod[0];
//    math::Spline::Point res;
//    float t = 1;
//    float dutyF = gaitGenerator->dutyFactor[0];
//    if (gaitGenerator->desiredLegState[0] == LegState::SWING) {
//        t = gaitGenerator->phaseInFullCycle[0] - dutyF;
//    } else if (gaitGenerator->desiredLegState[1] == LegState::SWING) {
//        t = gaitGenerator->phaseInFullCycle[1] - dutyF;
//    } else if (gaitGenerator->phaseInFullCycle[0]<gaitGenerator->phaseInFullCycle[1]) { // 0-leg just finished swinging
//        t = gaitGenerator->phaseInFullCycle[0] + (1-dutyF);
//    } else {
//        t = gaitGenerator->phaseInFullCycle[1] + (1-dutyF);
//    }
//    t *=2.0f;
//    Vec12<float> baseStartState = footholdPlanner->firstSwingBaseState;

//    for (u8 axis(0); axis<2; ++axis) {
//        // auto sPoint = math::Spline::Point(0, 0, 0);
//        // auto ePoint = math::Spline::Point(0.2, 0.1, 0);
//        auto sPoint = math::Spline::Point(baseStartState[axis], baseStartState[3+axis], 0.05);
//        auto ePoint = math::Spline::Point(comDestination[axis], v_des_world[axis], 0.05);
//        res.x = (1-t) *sPoint.x + t* ePoint.x;
//        res.xd = ePoint.xd;
//        // res.xd = ePoint.xd;
//        // auto spline = math::FifthOrderPolySpline(st, et, sPoint, ePoint);

//        // for (t = 0; t < et+0.01; t+=0.01) {

//        //     spline.getPoint(t, res);
//        //     // world_position_desired[axis] = res.x;
//        //     vis.datax.push_back(t);
//        //     vis.datay1.push_back(res.x);
//        //     vis.datay2.push_back(res.xd);
//        //     vis.datay3.push_back(res.xdd);//gaitGenerator->normalizedPhase[0]);
//        //     // vis.datay5.push_back(desiredP[0]);

//        // }

//        // vis.Show();
//        // exit(0);
//        // spline.getPoint(std::min(1.0f, t + 0.01f) *et, res);
//        // if (axis == 0) {
//            // vis.datay5.push_back(world_position_desired[0]);
//            // vis.datay4.push_back(res.xd);
//            //  vis.datay2.push_back(res.x);
//        // }
//        world_position_desired[axis] = res.x;
//        // std::cout << "[s]: " << sPoint << ", [e]: " << ePoint << ", [res]: " <<res <<std::endl;
//        v_des_world[axis] = res.xd;

//    }
    // vis.Show();

    // Vec3<float> V = robot->GetEstimatedVelocityInBaseFrame();
    // auto footPositionB =  robot->GetFootPositionsInBaseFrame();
    // auto footPositionW =  robot->GetFootPositionsInWorldFrame();

    // auto& fullModel = robot->model;
    //     auto motorV = robot->GetMotorVelocities();
    //     auto motorA = robot->GetMotorAngles();
    //     // auto motorddq = quadruped->motorddq;
    //     vis.datax.push_back(robot->GetTimeSinceReset());
    //     vis.datay1.push_back(robot->basePosition[0]);
    //     // vis.datay2.push_back(desiredFootholdsInWorldFrame(0,0));
    //     vis.datay2.push_back(_yaw_turn_rate);

    //     // vis.datay3.push_back(v_robot[0]);
    //     vis.datay3.push_back(_yaw_des_true);
    //     vis.datay4.push_back(robot->baseRollPitchYawRate[2]);
    //     // vis.datay5.push_back(world_position_desired[0]);
    //     vis.datay5.push_back(robot->baseRollPitchYaw[2]);



    // update mpc_table
    Vec4<float> progress = gaitGenerator->phaseInFullCycle;
    float dPhase = 1.0 / (numHorizonL*horizonLength);
    for(int i = 0; i < horizonLength; i++) {
        for(int j = 0; j < NumLeg; ++j) {
            float ithMPCPhase = progress[j] + i * dPhase;
            while (ithMPCPhase > 1.0)
            {
                ithMPCPhase -= 1.0;
            }
            // std::cout << "i=" << i<< ", j=" << j << ", ithPhase = " << ithMPCPhase << std::endl;
            if(ithMPCPhase < gaitGenerator->dutyFactor[j] || gaitGenerator->legState[j] == LegState::EARLY_CONTACT) {
                // if (ithMPCPhase > gaitGenerator->dutyFactor[j] - 0.1) {
                //     _mpcTable(i, j) = (gaitGenerator->dutyFactor[j] - ithMPCPhase)/0.1;
                // } else {
                    // _mpcTable(i, j) = ithMPCPhase - gaitGenerator->contactStartPhase[j];
                    // _mpcTable(i, j) = _mpcTable(i, j) > 0.2 ? 1 : std::max(0.2, _mpcTable(i, j)/0.2);
                // }
                _mpcTable(i, j) = 1; // this leg is in stance, mpc should consider this leg.
            } else {
                _mpcTable(i, j) = 0;
            }
        }
    }
    for (int legId = 0; legId < 4; ++legId) {
        _mpcTable(0, legId) = float(contactState[legId]);
    }

    // std::cout << "wp =" << world_position_desired.transpose()
    //             << "\n v_des_robot = " << v_des_robot.transpose()
    //             << "\n v_robot = " << v_robot.transpose()
    //             << "\n rpy_init = " << rpy_int.transpose()
    //             << "\n rpy_comp = " << rpy_comp.transpose() <<std::endl;

    updateMPCIfNeeded(robot, false);

    // Update For WBC
    // TODO: add WBC
//    if (useWBC) {
//        seResult.wbcData.allowAfterMPC = !myflags;
//        seResult.wbcData.pBody_des[0] = robot->basePosition[0];//world_position_desired[0];
//        seResult.wbcData.pBody_des[1] = robot->basePosition[1];//world_position_desired[1];
//        seResult.wbcData.pBody_des[2] = _body_height;

//        seResult.wbcData.vBody_des[0] = v_des_world[0];
//        seResult.wbcData.vBody_des[1] = v_des_world[1];
//        seResult.wbcData.vBody_des[2] = 0.;
//        // seResult.wbcData.vBody_des[2] = v_des_world[2];

//        seResult.wbcData.aBody_des.setZero();

//        seResult.wbcData.pBody_RPY_des[0] = rpy_comp[0];
//        seResult.wbcData.pBody_RPY_des[1] = rpy_comp[1];
//        seResult.wbcData.pBody_RPY_des[2] = _yaw_des_true;

//        seResult.wbcData.vBody_Ori_des[0] = 0.;
//        seResult.wbcData.vBody_Ori_des[1] = 0.;
//        seResult.wbcData.vBody_Ori_des[2] = _yaw_turn_rate;

//        seResult.wbcData.contact_state = contactState;
//    }
    iterationCounter++;
}

void qrMITConvexMPCStanceLegController::updateMPCIfNeeded(qrRobot *_quadruped, bool omniMode)
{
    // std::cout << "[MPCNEED] " << iterationCounter % iterationsInaMPC << std::endl;
    if (iterationCounter % (iterationsInaMPC/2) == 0 || iterationCounter < 50) {
    // if (iterationCounter % 2 == 0) {
        //auto& seResult = _quadruped->stateDataFlow;
        Vec3<float> p = _quadruped->GetBasePosition();

        // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};
        // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
        // x_comp_integral);

        const float max_posx_error = 0.1f;
        const float max_posy_error = 0.1f;
        float xStart = world_position_desired[0];
        float yStart = world_position_desired[1];

        if (xStart - p[0] > max_posx_error) xStart = p[0] + max_posx_error;
        if (p[0] - xStart > max_posx_error) xStart = p[0] - max_posx_error;

        if (yStart - p[1] > max_posy_error) yStart = p[1] + max_posy_error;
        if (p[1] - yStart > max_posy_error) yStart = p[1] - max_posy_error;

        world_position_desired[0] = xStart;
        world_position_desired[1] = yStart;

        Vec3<float> omega_des(0, 0, _yaw_turn_rate);
        // omega_des = seResult.baseRMat * omega_des; // base -> world // todo

        float trajInitial[12] = {rpy_comp[0], // 0
                                 rpy_comp[1], // 1
                                 _yaw_des_true,      // 2
                                 // yawStart,    // 2
                                 xStart,             // 3
                                 yStart,             // 4
                                 _body_height,// 5
                                //  world_position_desired[2],
                                 omega_des[0],                  // 6
                                 omega_des[1],                  // 7
                                 omega_des[2],     //_yaw_turn_rate,     // 8
                                 v_des_world[0],   // 9
                                 v_des_world[1],   // 10
                                 0.f};               // 11
                                //  v_des_world[2]};
        // for (int i=0; i<12; ++i) {
        //     std::cout << " , " << trajInitial[i];
        // }
        // std::cout << std::endl;
        for (int i = 0; i < horizonLength; ++i) {
            for (int j = 0; j < 12; ++j) trajAll[12 * i + j] = trajInitial[j];

            if (i == 0) // start at current position,  TODO consider not doing this
            {
                trajAll[2] = _yaw_des_true;
                // trajAll[2] = robot->GetBaseRollPitchYaw()[2];
            } else {
                trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC*v_des_world[0];// _x_vel_des;
                trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC *v_des_world[1];// _y_vel_des;
                // trajAll[12 * i + 5] = trajAll[12 * (i - 1) + 5] + dtMPC * v_des_world[2];
                // trajAll[12 * i + 1] = trajAll[12 * (i - 1) + 1] + dtMPC * omega_des[1];
                trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
            }
        }


        MITTimer solveTimer;
        // int cmpc_use_sparse = 0;
        // if (cmpc_use_sparse > 0.5) {
        //     solveSparseMPC(_quadruped);
        // } else {
            solveDenseMPC(_quadruped);
        // }
        // printf("MPC SOLVE TIME: %.3f\n", solveTimer.getMs());
        //seResult.visualizer.sa[3].Update(solveTimer.getMs());
        myflags = true;
    } else {
        myflags = false;
    }
}

void qrMITConvexMPCStanceLegController::solveDenseMPC(qrRobot *_quadruped)
{
    //auto& seResult = _quadruped->stateDataFlow;

    Vec3<float> rpy = _quadruped->GetBaseRollPitchYaw();
    Vec3<float> pos = _quadruped->GetBasePosition();
    Quat<float> quat = _quadruped->GetBaseOrientation();
    Eigen::Matrix<float,3,4> footPosInBaseFrame = _quadruped->state.GetFootPositionsInBaseFrame();
    float *weights = Q;
    // float alpha = 4e-5;// make setting eventually
    float alpha = 4e-6;// make setting eventually
    // float alpha = 4e-7; // make setting eventually: DH
    float *p = pos.data();
    Eigen::Matrix<float, 3, 1> vBase = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * robot->GetBaseVelocity();
    Eigen::Matrix<float, 3, 1> wBase = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * robot->GetBaseRollPitchYawRate();

    float *v = vBase.data();
    float *w = wBase.data();
    float *q = quat.data();

    // float r[12];
    Eigen::Matrix<float, 3, 4> foot2ComInWorldFrame = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * (footPosInBaseFrame.colwise() - _quadruped->config->comOffset);
    float *r = foot2ComInWorldFrame.data(); // colMajor
    // for (int i = 0; i < 12; i++)
    //     r[i] = pFoot(i / 4, i % 4) - pos[i/4];

    // if (alpha > 1e-4) {
    //     std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    //     alpha = 1e-5;
    // }

    // Vec3<float> pxy_act(p[0], p[1], 0);
    // Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
    // Vec3<float> pxy_err = pxy_act - pxy_des;
    float pz_err = p[2] - _body_height;
    Vec3<float> vxy(v[0], v[1], 0);

    // Timer t1;
    update_x_drag(x_comp_integral);

    float cmpc_x_drag = 3.0;
    if (vxy[0] > 0.3 || vxy[0] < -0.3) {
        // x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
        x_comp_integral += cmpc_x_drag * pz_err * dtMPC / vxy[0];
    }
    // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

    // MITTimer t2;
    update_problem_data_floats(p, v, q, w, r, rpy[2], weights, trajAll, alpha, _mpcTable.data());
    // printf("update_problem_data_floats time %f ms\n", t2.getMs());
    // seResult.visualizer.sa[3].Update(t2.getMs());

    for (int leg = 0; leg < NumLeg; ++leg) {
        for (int axis = 0; axis < 3; ++axis) {
            f(axis, leg) = get_solution(leg * 3 + axis);
        }
        f_ff.col(leg) = -math::quaternionToRotationMatrix(robot->GetBaseOrientation()) * f.col(leg);
        //seResult.wbcData.Fr_des[leg] = f.col(leg);
    }
}

void qrMITConvexMPCStanceLegController::solveSparseMPC(qrRobot *_quadruped)
{
    // X0, contact trajectory, state trajectory, feet, get result!
    //auto& seResult = _quadruped->stateDataFlow;

    std::vector<Vec4<bool>> contactStates;
    for (int i = 0; i < horizonLength; ++i) {
        contactStates.emplace_back(_mpcTable(i, 0), _mpcTable(i, 1),
                                   _mpcTable(i, 2), _mpcTable(i, 3));
    }

    for (int i = 0; i < horizonLength; ++i) {
        for (u32 j = 0; j < 12; ++j) {
            _sparseTrajectory[i][j] = trajAll[i * 12 + j];
        }
    }
    Vec3<float> rpy = _quadruped->GetBaseRollPitchYaw();
    Vec3<float> pos = _quadruped->GetBasePosition();
    Quat<float> quat = _quadruped->GetBaseOrientation();
    Vec12<float> feet;
    for (u32 foot = 0; foot < NumLeg; ++foot) {
        for (u32 axis = 0; axis < 3; ++axis) {
            feet[foot * 3 + axis] = pFoot(axis,foot) - pos[axis];
        }
    }

    Vec3<float> vWorld = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * robot->GetBaseVelocity();
    Vec3<float> wWorld = math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose() * robot->GetBaseRollPitchYawRate();
    _sparseCMPC.setX0(pos,  vWorld, quat, wWorld);
    _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
    _sparseCMPC.setStateTrajectory(_sparseTrajectory);
    _sparseCMPC.setFeet(feet);
    _sparseCMPC.run();

    Vec12<float> resultForce = _sparseCMPC.getResult();
    Mat3<float> baseRMat =  math::quaternionToRotationMatrix(robot->GetBaseOrientation()).transpose();
    for (u32 foot = 0; foot < 4; foot++) {
        Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
        // printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
        f_ff.col(foot) = -baseRMat.transpose() * force; // in base frame
        //seResult.wbcData.Fr_des[foot] = force; // in world frame
    }
}

void qrMITConvexMPCStanceLegController::initSparseMPC()
{
    double maxForce = 120;
    double mass = robot->GetBodyMass();
    Eigen::Matrix<double, 3, 3> inertia = robot->config->bodyInertia.cast<double>();
    std::vector<double> dtTraj;
    for (int i = 0; i < horizonLength; i++) {
        dtTraj.push_back(dtMPC);
    }

    Vec12<double> weights;
    weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
    // weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

    _sparseCMPC.setRobotParameters(inertia, mass, maxForce);
    _sparseCMPC.setFriction(1.0);
    _sparseCMPC.setWeights(weights, 4e-5);
    _sparseCMPC.setDtTrajectory(dtTraj);
    _sparseTrajectory.resize(horizonLength);
}

// TODO: walk locomotion has been deleted
void qrMITConvexMPCStanceLegController::UpdateDesCommand()
{
    Eigen::Matrix<float, 3, 1> robotComPosition;
    Eigen::Matrix<float, 3, 1> robotComVelocity;
    Eigen::Matrix<float, 3, 1> robotComRpy;
    Eigen::Matrix<float, 3, 1> robotComRpyRate;
    Eigen::Matrix<float, 6, 1> robotQ;
    Eigen::Matrix<float, 6, 1> robotDq;

    Eigen::Matrix<float, 3, 1> desiredComPosition(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 3, 1> desiredComVelocity(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 3, 1> desiredComRpy(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 3, 1> desiredComAngularVelocity(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 6, 1> desiredQ;
    Eigen::Matrix<float, 6, 1> desiredDq;
    Eigen::Matrix<float, 6, 1> desiredDdq;

    UpdateFRatio(contacts, N, moveBasePhase);

    Eigen::Matrix<float, 3, 4> footPoseWorld = robot->state.GetFootPositionsInWorldFrame();

    Vec6<float> pose;
    Vec6<float> twist; // v, wb

    pose.head(3) = robot->GetBasePosition();
    // pose[0] = max(pose[0], robot->GetBasePosition()[0]);

    Eigen::Matrix<int, 3, 4> jointIdxs;
    Eigen::Matrix<int, 3, 1> jointIdx;
    Eigen::Matrix<float, 3, 1> jointAngles;
    Eigen::Matrix<float, 3, 4> com2FootInWorld = footPoseWorld.colwise() - pose.head(3);

    Quat<float> robotComOrientation = robot->GetBaseOrientation();
    Mat3<float> Rb = math::quaternionToRotationMatrix(robotComOrientation).transpose();
    Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
    Mat3<float> Rc = math::quaternionToRotationMatrix(controlFrameOrientation).transpose();
    Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> Rcb = Rc.transpose() * Rb;


    /// current robot status  ///
    robotComRpy = robot->GetBaseRollPitchYaw(); // world frame
    // std::cout << "robotComRpy = " << robotComRpy << std::endl;
    robotComVelocity = robotEstimator->GetEstimatedVelocity();  // base frame
    robotComRpyRate = robot->GetBaseRollPitchYawRate();  // base frame
    // std::cout <<  "mode = " << robot->controlParams["mode"] << std::endl;

    //TODO: currently, set world frame to true
    bool computeForceInWorldFrame = true;
    if (computeForceInWorldFrame) {
        robotComPosition = {0., 0., robot->GetBasePosition()[2]}; // vel mode in base frame, height is in world frame.
        // robotComRpy[2] = 0.f;
        robotComVelocity = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
        robotComRpyRate = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
    } else { // in control frame
        robotComPosition = {0., 0., robot->GetBasePosition()[2]}; // vel mode in base frame, height is in world frame.
        robotComRpy[2] = 0.f;
        if (groundEstimator->terrain.terrainType>=2) { // not horizontal plane
            robotComPosition = math::TransformVecByQuat(math::quatInverse(controlFrameOrientation), robotComPosition);
            robotComPosition[0] = 0.f;
            robotComPosition[1] = 0.f;
            robotComPosition = {0., 0., robot->config->bodyHeight}; // vel mode in base frame, height is in world frame.

            robotComVelocity = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
            robotComVelocity = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComVelocity); // in control frame

            robotComRpy = math::rotationMatrixToRPY(Rcb.transpose()); // body orientation in control frame.
            robotComRpyRate = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
            robotComRpyRate = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComRpyRate); // in control frame
        }
    }

    // robotComPosition = math::invertRigidTransform({0,0,0}, robotComOrientation, robotComPosition); // in world frame
    // robotComPosition = math::RigidTransform({0,0,0}, controlFrameOrientation, robotComPosition); // in control frame

    robotQ << robotComPosition, robotComRpy;//world frame
    robotDq << robotComVelocity, robotComRpyRate;
    // std::cout << "[torque stance]: des command = " << desiredStateCommand->stateDes << std::endl;
    /// desired robot status  ///
    if (computeForceInWorldFrame) {
        auto &comAdjPosInBaseFrame = comPlanner->GetComPosInBaseFrame();
        Vec3<float> newComPosInWorldFrame = Rb * comAdjPosInBaseFrame + robot->GetBasePosition();
        desiredComPosition = {newComPosInWorldFrame[0], newComPosInWorldFrame[1], stateDes(2)}; // todo

        // desiredComPosition << 0, 0, desiredStateCommand->stateDes(2);
        float pitch = groundRPY[1];
        float pitchMax = 0.5;
        if (abs(pitch) < 0.1) {
            pitch = 0;
        } else if (pitch > pitchMax) {
            pitch = pitchMax;
        } else if (pitch < -pitchMax) {
            pitch = -pitchMax;
        }
        desiredComRpy << 0.f, pitch, 0.f;
        // std::cout << "pitch = " << pitch << std::endl;
        Eigen::Matrix<float,3,4> footPosInBaseFrame = robot->state.GetFootPositionsInBaseFrame();
        float scaleFactor = 1;
        // Eigen::Matrix<float,3,4> com2FootInWorldFrame = footPoseWorld.colwise() - robot->basePosition;
        float footX = std::min(footPosInBaseFrame(0, 0), footPosInBaseFrame(0, 1));
        if (footX < 0.1) {
            scaleFactor = std::max(0.1f, footX / 0.1f);
        }
        // std::cout << "scaleFactor = " << scaleFactor << std::endl;
        // is com in support polygon ?
        // todo

        desiredComVelocity = (scaleFactor * stateDes.segment(6,3)); // in base
        if (pitch < 0.1 && desiredComVelocity[2] > 0.01) {
            desiredComPosition[2] += 0.04 * abs(pitch/pitchMax); // todo
        }
        desiredComAngularVelocity = stateDes.segment(9,3); // in base
    } else { // in control frame
       desiredComPosition << 0.f, 0.f, desiredBodyHeight*std::abs(std::cos(groundRPY[1]));
       desiredComPosition[2] = robotComPosition[2]*0.7 + desiredComPosition[2]*0.3;
       desiredComRpy << -groundRPY[0], 0.f, -groundRPY[2]; // not control roll/yaw
       desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f}; // in base/control frame
       desiredComVelocity = Rc * desiredComVelocity;
       desiredComAngularVelocity = {0.f, 0.f, desiredTwistingSpeed};
    }

    desiredQ << desiredComPosition, desiredComRpy;

    //std::cout << "desiredQ" << desiredQ.transpose() << std::endl;
    //std::cout << "robotQ" << robotQ.transpose() << std::endl;
    Vec6<float> dq = desiredQ - robotQ;
    desiredDq << desiredComVelocity, desiredComAngularVelocity;
    Vec6<float> ddq = desiredDq - robotDq;
    //std::cout << "desiredDq" << desiredDq.transpose() << std::endl;
    //std::cout << "robotDq" << robotDq.transpose() << std::endl;

//   if (computeForceInWorldFrame && robot->controlParams["mode"]!=LocomotionMode::ADVANCED_TROT) {
//       // dq
//       // method 1: R(dR^T-->rpy)
//       Mat3<float> robotR = math::rpyToRotMat(robotComRpy).transpose();
//       Mat3<float> desiredRobotRT = math::rpyToRotMat(desiredComRpy);
//       Mat3<float> dR = desiredRobotRT*robotR;
//       dq.tail(3) = robotR * math::rotationMatrixToRPY(dR);
//       //  method 2:   (R*dR)^T --> rpy
//       // dq.tail(3) = math::rotationMatrixToRPY(robotR * dR);

//       // ddq
//       // method 1 : R*((/hat(WBdes) - /hat(WBcurr))---> to skewV)
//       Mat3<float> RTWdes = math::vectorToSkewMat(desiredRobotRT *  desiredComAngularVelocity);
//       Mat3<float> RTWcur = math::vectorToSkewMat(robotR.transpose() *  robotComRpyRate);
//       Vec3<float> dw = robotR * math::matToSkewVec(RTWdes - RTWcur);
//       ddq.tail(3) =  dw;
//       // method 2: do nothing
//   } else if(groundEstimator->terrain.terrainType >= 2) { // not horizontal ground, computeForceInControlFrame
//       // dq.tail(3) = Rcb * dq.tail(3); // todo, input should not be represent in base frame
//   }

    desiredDdq = KP.cwiseProduct(dq) + KD.cwiseProduct(ddq);
    desiredDdq = desiredDdq.cwiseMin(maxDdq).cwiseMax(minDdq); // Clip
    // std::cout << "desiredDdq" << desiredDdq.transpose() << std::endl;

    stateDes << desiredQ, desiredDq;

    stateCur << robotQ, robotDq;

    std::cout << "[MPC desired state] " << stateDes.transpose() << std::endl;
    std::cout << "[MPC current state] " << stateCur.transpose() << std::endl;
    //ddqDes.head(6) = desiredDdq;
    // std::cout << "contact for force compute " << contacts.transpose() << std::endl;
}

void qrMITConvexMPCStanceLegController::UpdateControlParameters(const Eigen::Vector3f& linSpeed, const float &angSpeed)
{
    qrStanceLegController::UpdateControlParameters(linSpeed, angSpeed);

    float filterFactor = 0.02f;

    stateDes.segment(6, 3) = stateDes.segment(6, 3) * (1.0f - filterFactor) + linSpeed * filterFactor;
    stateDes(11) = stateDes(11) * (1.0f - filterFactor) + angSpeed * filterFactor;
}

void qrMITConvexMPCStanceLegController::initStateDes()
{
    std::cout << " MPC: desired speed: " << desiredSpeed <<std::endl;
    stateDes.segment(6, 3) = desiredSpeed;
    stateDes(0) = dt * stateDes(6);
    stateDes(1) = dt * stateDes(7);
    stateDes(2) = desiredBodyHeight;
    stateDes(3) = 0.0f;
    stateDes(4) = 0.0f;
    stateDes(5) = dt * desiredTwistingSpeed;
    stateDes(9) = 0.0f;
    stateDes(10) = 0.0f;
    stateDes(11) = desiredTwistingSpeed;
}
