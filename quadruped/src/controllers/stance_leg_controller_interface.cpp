/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: the interface of all stance leg controllers.
* Author: Zhu Yijie
* Create: 2022-05-01
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "controllers/stance_leg_controller_interface.h"

namespace Quadruped {

    StanceLegControllerInterface::StanceLegControllerInterface(Robot *robot,
                                                         GaitGenerator *gaitGenerator,
                                                         StateEstimatorContainer<float>* stateEstimators,
                                                         ComAdjuster *comAdjuster,
                                                         PosePlanner *posePlanner,
                                                         FootholdPlanner *footholdPlanner,
                                                         UserParameters& userParameters,
                                                         std::string configFilepath
                                                         )
    {
        c = nullptr;
        c1 = new TorqueStanceLegController(robot,
                                            gaitGenerator,
                                            stateEstimators,
                                            comAdjuster,
                                            posePlanner,
                                            footholdPlanner,
                                            userParameters, 
                                            configFilepath);
        c2 = new MITConvexMPCStanceLegController(robot,
                                            gaitGenerator,
                                            stateEstimators,
                                            comAdjuster,
                                            posePlanner,
                                            footholdPlanner,
                                            userParameters, 
                                            configFilepath);
        
        if (robot->controlParams["mode"]==LocomotionMode::ADVANCED_TROT) {
            c = c2;
        } else {
            std::cout << "[STANCE CONTROLLER INTERFACE] use TorqueStanceLegController" <<std::endl;
            // TorqueStanceLegController, TorqueStanceLegControllerMPC
            c = c1;
        }
        
    }

    StanceLegControllerInterface::~StanceLegControllerInterface()
    {
        if (c1) {
            delete c1;
        }
        if (c2) {
            delete c2;
        }
        c = nullptr;
        c1 = nullptr;
        c2 = nullptr;
    }

    void StanceLegControllerInterface::Reset(float currentTime_)
    {
        if (c->robot->controlParams["mode"] != LocomotionMode::ADVANCED_TROT) {
            c = c1;
        } else {
            c = static_cast<MITConvexMPCStanceLegController*>(c2);
        }
        c->Reset(currentTime_);
    }

    void StanceLegControllerInterface::Update(float currentTime_)
    {
        c->Update(currentTime_);
    }

    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> StanceLegControllerInterface::GetAction()
    {
        return c->GetAction();
    }

} // namespace Quadruped
