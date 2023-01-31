/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*       add computing desired acceleration in world frame @ Zhu Yijie & Zang 2022-02-01;
*       add controlModeStr @ Zhu Linsen 2022-03-01;
*       add UpdateFRatio @ Zhu Yijie 2022-03-01;
*       add UpdateDesCommand @ Zhu Yijie 2022-04-01;
*/

#ifndef ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H

#include "utils/se3.h"
#include "robots/robot.h"
#include "gait/openloop_gait_generator.h"
#include "estimators/state_estimator.hpp"
#include "planner/com_adjuster.h"
#include "planner/pose_planner.h"
#include "planner/foothold_planner.h"
#include "controllers/desired_state_command.hpp"

namespace Quadruped {
    /**
     * @brief Control stance leg of robot
     */
    class TorqueStanceLegController {
    public:
        TorqueStanceLegController(Robot *robot,
                                  GaitGenerator *gaitGenerator,
                                  StateEstimatorContainer<float>* stateEstimators,
                                  ComAdjuster *comAdjuster,
                                  PosePlanner *posePlanner,
                                  FootholdPlanner *footholdPlanner,
                                  UserParameters& userParameters,
                                  std::string configFilepath
                                  );

        virtual ~TorqueStanceLegController() = default;

        TorqueStanceLegController() {};

        void BindCommand(DesiredStateCommand* desiredStateCommandIn)
        {
            desiredStateCommand = desiredStateCommandIn;
        }
        
        virtual void Reset(float currentTimeIn);

        /**
         * @brief update the ratio of the friction force to robot gravity
         * @param contacts Vec4<bool>&,  descripte the contact status of four feet
         * @param N int&, the number of contact feet
         * @param normalizedPhase float&, the phase of swing leg
         */
        void UpdateFRatio(Vec4<bool> &contacts, int &N, float &normalizedPhase);

        virtual void UpdateDesCommand();
        
        void Update(float currentTime);

        virtual std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

//private:
        float currentTime;
        Robot *robot;
        GaitGenerator *gaitGenerator;
        RobotEstimator *robotEstimator;
        GroundSurfaceEstimator *groundEstimator;
        ComAdjuster *comAdjuster;
        PosePlanner *posePlanner;                          
        FootholdPlanner *footholdPlanner;
        Eigen::Matrix<float, 3, 1> desiredSpeed;
        float desiredTwistingSpeed;
        float desiredBodyHeight = 0.3; // overwrite in the class constructor by userParameters.desiredHeight
        Vec4<float> frictionCoeffs;
        YAML::Node param;
        std::string controlModeStr;
        DesiredStateCommand* desiredStateCommand;
        bool computeForceInWorldFrame;
        int force_dim;
        int N; // number of stance legs at current time
        float moveBasePhase;
        Eigen::Matrix<bool, 4, 1> contacts;
        
        Eigen::Matrix<float, 6, 1> KP; // for acceleration
        Eigen::Matrix<float, 6, 1> KD; // for acceleration
        Eigen::Matrix<float, 6, 1> maxDdq;
        Eigen::Matrix<float, 6, 1> minDdq;
        Eigen::Matrix<float, 6, 1> accWeight;
        Vec4<float> fMinRatio; // the minimum ratio
        Vec4<float> fMaxRatio; // the maximum ratio
        // for debug
        long long count = 0;
        Eigen::Matrix<float,3,4> lastMotorTorques = Eigen::Matrix<float,3,4>::Zero();

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H
