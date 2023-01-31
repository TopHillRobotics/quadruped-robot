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

#include "controllers/balance_controller/torque_stance_leg_controller.h"
#include "controllers/mpc/qp_torque_optimizer_mpc.h"

namespace Quadruped {
    /** @brief  
     * Our convex mpc impelement
     */
    class TorqueStanceLegControllerMPC : public TorqueStanceLegController {
    public:
        TorqueStanceLegControllerMPC(Robot *robot,
                                    GaitGenerator *gaitGenerator,
                                    StateEstimatorContainer<float>* stateEstimators,
                                    ComAdjuster *comAdjuster,
                                    PosePlanner *posePlanner,
                                    FootholdPlanner *footholdPlanner,
                                    UserParameters& userParameters,
                                    std::string configFilepath
                                    );

        virtual ~TorqueStanceLegControllerMPC() = default;

        virtual std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

        
        int mpcStep = 0;
        const int ctrlHorizon = 1;
        Eigen::Matrix<float, 3*2, 4> contactForcesMPC;
        Eigen::Matrix<float, 13, 1> XWeight;

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_MPC_H