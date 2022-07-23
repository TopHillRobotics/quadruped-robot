/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua
* Create: 2021-12-13
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#ifndef ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_MPC_H
#define ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_MPC_H

#include "mpc_controller/torque_stance_leg_controller.h"
#include "mpc_controller/qp_torque_optimizer_mpc.h"
namespace Quadruped {
    class TorqueStanceLegControllerMPC : public TorqueStanceLegController {
    public:
        TorqueStanceLegControllerMPC(Robot *robot,
                                    OpenloopGaitGenerator *gaitGenerator,
                                    RobotEstimator *robotEstimator,
                                    GroundSurfaceEstimator *groundEstimatorIn,
                                    ComAdjuster *comAdjuster,
                                    PosePlanner *posePlanner,
                                    FootholdPlanner *footholdPlanner,
                                    Eigen::Matrix<float, 3, 1> desired_speed,
                                    float desiredTwistingSpeed,
                                    float desiredBodyHeight,
                                    int numLegs,
                                    std::string configFilepath,
                                    std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45});

        virtual ~TorqueStanceLegControllerMPC() = default;

        virtual std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

        
        int mpcStep = 0;
        const int ctrlHorizon = 1;
        Eigen::Matrix<float, 3*2, 4> contactForcesMPC;
        Eigen::Matrix<float, 13, 1> XWeight;

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_MPC_H