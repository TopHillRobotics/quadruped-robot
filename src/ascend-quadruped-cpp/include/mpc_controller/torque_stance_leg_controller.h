/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#ifndef ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H

#include "utils/se3.h"
#include "robots/robot.h"
#include "mpc_controller/openloop_gait_generator.h"
#include "state_estimator/robot_estimator.h"
#include "state_estimator/ground_estimator.h"
#include "planner/com_adjuster.h"
#include "planner/pose_planner.h"
#include "planner/foothold_planner.h"

namespace Quadruped {
    /**
     * @brief Control stance leg of robot
     */
    class TorqueStanceLegController {
    public:
        TorqueStanceLegController(Robot *robot,
                                  OpenloopGaitGenerator *gaitGenerator,
                                  RobotEstimator *robotVelocityEstimator,
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

        virtual ~TorqueStanceLegController() = default;

        void Reset(float currentTime);

        /**
         * @brief update the ratio of the friction force to robot gravity
         * @param contacts Vec4<bool>&,  descripte the contact status of four feet
         * @param N int&, the number of contact feet
         * @param normalizedPhase float&, the phase of swing leg
         */
        void UpdateFRatio(Vec4<bool> &contacts, int &N, float &normalizedPhase);

        void Update(float currentTime);

        virtual std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

//private:
        Robot *robot;
        OpenloopGaitGenerator *gaitGenerator;
        RobotEstimator *robotEstimator;
        GroundSurfaceEstimator *groundEstimator;
        ComAdjuster *comAdjuster;
        PosePlanner *posePlanner;                          
        FootholdPlanner *footholdPlanner;
        Eigen::Matrix<float, 3, 1> desiredSpeed = {0., 0., 0.};
        float desiredTwistingSpeed = 0.;
        float desiredBodyHeight = 0.45; //overwrite in the class constructor by robot->bodyHeight
        int numLegs = 4;
        std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45};
        std::string configFilepath;

        int force_dim;
        Eigen::Matrix<float, 6, 1> KP;
        Eigen::Matrix<float, 6, 1> KD;
        Eigen::Matrix<float, 6, 1> maxDdq;
        Eigen::Matrix<float, 6, 1> minDdq;
        Eigen::Matrix<float, 6, 1> accWeight;
        Vec4<float> fMinRatio; // the minimum ratio
        Vec4<float> fMaxRatio; // the maximum ratio
        
        float currentTime;
        long long count = 0;

        const int n = 20000;
        std::vector<float> datax, datay1, datay2, datay3, datay4;
        Eigen::Matrix<float,3,4> lastMotorTorques = Eigen::Matrix<float,3,4>::Zero();

    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H
