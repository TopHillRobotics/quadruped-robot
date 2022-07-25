/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22
*/

#ifndef ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H

#include "robots/motor.h"
#include "openloop_gait_generator.h"
#include "state_estimator/robot_estimator.h"
#include "state_estimator/ground_estimator.h"
#include "planner/foothold_planner.h"
#include "mpc_controller/foot_trajectory_generator.h"

namespace Quadruped {
    class qrSwingLegController {

    public:
        qrSwingLegController(Robot *robot,
                                  qrGaitGenerator *gaitGenerator,
                                  qrRobotEstimator *stateEstimator,
                                  qrGroundSurfaceEstimator *groundEstimator,
                                  qrFootholdPlanner *FootholdPlanner,
                                  Eigen::Matrix<float, 3, 1> desiredSpeed,
                                  float desiredTwistingSpeed,
                                  float desiredHeight,
                                  float footClearance,
                                  std::string configPath);

        ~qrSwingLegController() = default;

        /**
         * @brief Quadratic interpolation function, used to generate polygon curve.
         * @param phase
         * @param start
         * @param mid
         * @param end
         * @return a float value with phase
         */
        float GenParabola(float phase, float start, float mid, float end);

        /**
         * @brief Generating the trajectory of the swing leg
         * @param inputPhase
         * @param startPos
         * @param endPos
         * @return foot position like (x,y,z)
         */
        Eigen::Matrix<float, 3, 1> GenSwingFootTrajectory(float inputPhase,
                                                          Eigen::Matrix<float, 3, 1> startPos,
                                                          Eigen::Matrix<float, 3, 1> endPos);

        void Reset(float currentTime);

        void Update(float currentTime);

        /** @brief google's function for velocity mode control */
        std::map<int, Eigen::Matrix<float, 5, 1>> GetAction();

        Eigen::Matrix<float, 3, 1> desiredSpeed; // appear in velocity mode usually.
        float desiredTwistingSpeed; // appear in velocity mode usually.

        Robot *robot;
        qrGaitGenerator *gaitGenerator;
        qrRobotEstimator *stateEstimator;
        qrGroundSurfaceEstimator *groundEstimator;
        qrFootholdPlanner *footholdPlanner;
        Eigen::Matrix<int, 4, 1> lastLegState;
        Eigen::Matrix<float, 3, 1> desiredHeight;
        std::map<int, float> swigJointAngles;
        std::map<int, std::tuple<float, float, int>> swingJointAnglesVelocities;
        Eigen::Matrix<float, 3, 4> phaseSwitchFootLocalPos; // foot positions in base frame when switch leg state
        Eigen::Matrix<float, 3, 4> phaseSwitchFootGlobalPos; //foot positions in world frame when switch leg state
        Eigen::Matrix<float, 3, 4> footHoldInWorldFrame; //footholds in world frame
        Quat<float> controlFrameOrientationSource;
        Vec3<float> controlFrameOriginSource;
        Eigen::Matrix<float, 3, 4> phaseSwitchFootControlPos;
        Eigen::Matrix<float, 3, 4> footHoldInControlFrame;
        
        qrSwingFootTrajectory swingFootTrajectories[4];
        // yaml config
        YAML::Node swingLegConfig;
        // init pose in position mode
        std::vector<std::vector<float>> footInitPose;
        float footOffset;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H
