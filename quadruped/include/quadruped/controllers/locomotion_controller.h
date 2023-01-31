/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: a interface of robot locomotion controller.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_LOCOMOTION_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_LOCOMOTION_CONTROLLER_H

#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "utils/cppTypes.h"
#include "robots/robot.h"
#include "gait/openloop_gait_generator.h"
#include "gait/walk_gait_generator.h"
#include "controllers/swing_leg_controller.h"
#include "controllers/stance_leg_controller_interface.h"
#include "planner/com_adjuster.h"
#include "planner/pose_planner.h"
#include "estimators/state_estimator.hpp"

namespace Quadruped {
    /** 
     * @brief Universe Controller that combines planners and controllers.
     */
    class LocomotionController {

    public:
        LocomotionController(Robot *robot,
                             GaitGenerator *gaitGenerator,
                             DesiredStateCommand* desiredStateCommand,
                             StateEstimatorContainer<float> *stateEstimator,
                             ComAdjuster *comAdjuster,
                             PosePlanner *posePlanner,
                             RaibertSwingLegController *swingLegController,
                             StanceLegControllerInterface *stanceLegController,
                             UserParameters *userParameters);

        ~LocomotionController() = default;

        void Reset();

        void BindCommand()
        {
            swingLegController->BindCommand(desiredStateCommand);
            stanceLegController->BindCommand(desiredStateCommand);
        }

        void Update();

        /** @brief Compute all motors' commands via subcontrollers.
         *  @return tuple<map, Matrix<3,4>> : return control ouputs (e.g. positions/torques) for all (12) motors.
         */
        std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();
        std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> GetFakeAction();
    
        inline GaitGenerator *GetGaitGenerator()
        {
            return gaitGenerator;
        }

        inline RaibertSwingLegController *GetSwingLegController()
        {
            return swingLegController;
        }

        inline TorqueStanceLegController *GetStanceLegController()
        {
            return stanceLegController->c;
        }
        
        inline PosePlanner *GetPosePlanner()
        {
            return posePlanner;
        }

        double GetTime()
        {
            return robot->GetTimeSinceReset();
        }

        // let robot walk one step more.
        void ForwardOne();

        RaibertSwingLegController *swingLegController;
        StanceLegControllerInterface *stanceLegController;
        bool stop=false;
        int swingSemaphore = 10000000; // total steps 
        float stopTick = 0;
    private:
        Robot *robot;
        GaitGenerator *gaitGenerator;
        StateEstimatorContainer<float> *stateEstimator;
        ComAdjuster *comAdjuster;
        PosePlanner *posePlanner;
        DesiredStateCommand* desiredStateCommand;
        std::vector<MotorCommand> action;
        double resetTime;
        double timeSinceReset;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_LOCOMOTION_CONTROLLER_H
