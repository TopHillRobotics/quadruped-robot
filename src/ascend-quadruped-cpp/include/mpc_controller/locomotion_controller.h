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
#include "robots/timer.h"
#include "utils/cppTypes.h"
#include "robots/robot.h"
#include "robots/motor.h"
#include "mpc_controller/openloop_gait_generator.h"
#include "mpc_controller/walk_gait_generator.h"
#include "mpc_controller/raibert_swing_leg_controller.h"
#include "mpc_controller/torque_stance_leg_controller.h"
#include "mpc_controller/torque_stance_leg_controller_mpc.h"
#include "planner/com_adjuster.h"
#include "planner/pose_planner.h"
#include "state_estimator/robot_estimator.h"
#include "state_estimator/ground_estimator.h"

namespace Quadruped {
/** 
 * @brief Universe Controller that combines planners and estimators.
 * todo : laterly the estimators need to be moved outside and runs asynchronously with controllers.
 */
    class LocomotionController {

    public:
        LocomotionController(Robot *robot,
                             OpenloopGaitGenerator *gaitGenerator,
                             RobotEstimator *stateEstimator,
                             GroundSurfaceEstimator *groundEstimator,
                             ComAdjuster *comAdjuster,
                             PosePlanner *posePlanner,
                             RaibertSwingLegController *swingLegController,
                             TorqueStanceLegController *stanceLegController);

        ~LocomotionController() = default;

        void Reset();

        void Update();

        /** @brief Compute all motors' commands via subcontrollers.
         *  @return tuple<map, Matrix<3,4>> : return control ouputs (e.g. positions/torques) for all (12) motors.
         */
        std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();
        std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> GetFakeAction();
    
        inline OpenloopGaitGenerator *GetGaitGenerator()
        {
            return gaitGenerator;
        }

        inline RaibertSwingLegController *GetsSwingLegController()
        {
            return swingLegController;
        }

        inline TorqueStanceLegController *GetStanceLegController()
        {
            return stanceLegController;
        }

        inline RobotEstimator *GetRobotEstimator()
        {
            return stateEstimator;
        }

        inline GroundSurfaceEstimator *GetGroundEstimator()
        {
            return groundEstimator;
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
        TorqueStanceLegController *stanceLegController;
        bool stop=false;
        int swingSemaphore = 10000000; // total steps 
        float stopTick = 0;
    private:
        Robot *robot;
        OpenloopGaitGenerator *gaitGenerator;
        RobotEstimator *stateEstimator;
        GroundSurfaceEstimator *groundEstimator;
        ComAdjuster *comAdjuster;
        PosePlanner *posePlanner;
        std::vector<MotorCommand> action;
        double resetTime;
        double timeSinceReset;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_LOCOMOTION_CONTROLLER_H
