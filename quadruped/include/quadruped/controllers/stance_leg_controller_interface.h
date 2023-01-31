/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: the interface of all stance leg controllers.
* Author: Zhu Yijie
* Create: 2022-05-01
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_STANCE_LEG_CONTROLLER_INTERFACE_H
#define ASCEND_QUADRUPED_CPP_STANCE_LEG_CONTROLLER_INTERFACE_H

#include "controllers/balance_controller/torque_stance_leg_controller.h"
#include "controllers/mpc/torque_stance_leg_controller_mpc.h"
#include "controllers/mpc/mit_mpc_stance_leg_controller.h"

namespace Quadruped {
    /**
     * @brief Control stance leg of robot
     */
    class StanceLegControllerInterface {
    public:
        StanceLegControllerInterface(Robot *robot,
                                  GaitGenerator *gaitGenerator,
                                  StateEstimatorContainer<float>* stateEstimators,
                                  ComAdjuster *comAdjuster,
                                  PosePlanner *posePlanner,
                                  FootholdPlanner *footholdPlanner,
                                  UserParameters& userParameters,
                                  std::string configFilepath
                                  );

        ~StanceLegControllerInterface();

        void BindCommand(DesiredStateCommand* desiredStateCommandIn)
        {
            c->BindCommand(desiredStateCommandIn);
        }
        
        void Reset(float currentTime);
        
        void Update(float currentTime);

        std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

        
        TorqueStanceLegController* c, *c1, *c2;
         
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_STANCE_LEG_CONTROLLER_INTERFACE_H
