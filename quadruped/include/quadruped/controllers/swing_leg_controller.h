/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22
* Modify: mv foothold plan to FootHoldPlanner class. @ Zhu Yijie 2022.04.01
*/

#ifndef ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H

#include "robots/motor.h"
#include "gait/openloop_gait_generator.h"
#include "estimators/state_estimator.hpp"
#include "planner/foothold_planner.h"
#include "controllers/foot_trajectory_generator.h"

namespace Quadruped {
    class RaibertSwingLegController {

    public:
        RaibertSwingLegController(Robot *robot,
                                  GaitGenerator *gaitGenerator,
                                  StateEstimatorContainer<float>* stateEstimators,
                                  FootholdPlanner *FootholdPlanner,
                                  UserParameters& userParameters,
                                  std::string configPath);

        ~RaibertSwingLegController() = default;

        void BindCommand(DesiredStateCommand* desiredStateCommandIn)
        {
            desiredStateCommand = desiredStateCommandIn;
        }

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
        
        
        //
        DesiredStateCommand* desiredStateCommand;
        Eigen::Matrix<float, 3, 1> desiredSpeed; // appear in velocity mode usually.
        float desiredTwistingSpeed; // appear in velocity mode usually.
        
        // The position correction coefficients in Raibert's formula.
        // Kp = sqrt(z0/||g||)
        // const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03);
        Eigen::Matrix<float, 3, 1> swingKp;

        Robot *robot;
        GaitGenerator *gaitGenerator;
        RobotEstimator *stateEstimator;
        GroundSurfaceEstimator *groundEstimator;
        FootholdPlanner *footholdPlanner;
        UserParameters* userParameters;
        std::vector<u8> swingFootIds;
        Eigen::Matrix<int, 4, 1> lastLegState;
        Eigen::Matrix<float, 3, 1> desiredHeight;
        std::map<int, float> swigJointAngles;
        std::map<int, std::tuple<float, float, int>> swingJointAnglesVelocities;
        Eigen::Matrix<float, 3, 4> phaseSwitchFootLocalPos; // foot positions in base frame when switch leg state
        Eigen::Matrix<float, 3, 4> phaseSwitchFootGlobalPos; //foot positions in world frame when switch leg state
        Eigen::Matrix<float, 3, 4> footHoldInWorldFrame; //footholds in world frame
        Eigen::Matrix<float, 3, 4> footTargetPositionsInWorldFrame;
        Eigen::Matrix<float, 3, 4> desiredFootPositionsInBaseFrame;
        Quat<float> controlFrameOrientationSource;
        Vec3<float> controlFrameOriginSource;
        Eigen::Matrix<float, 3, 4> phaseSwitchFootControlPos;
        Eigen::Matrix<float, 3, 4> footHoldInControlFrame;
        
        SwingFootTrajectory swingFootTrajectories[4];
        SplineInfo splineInfo;
        // yaml config
        YAML::Node swingLegConfig;
        // init pose in position mode
        std::vector<std::vector<float>> footInitPose;
        float footOffset;
        
        int count =0;
        Vec3<float> dataRecord;

        Eigen::Matrix<float,3,4> foot_forces_kin;
        Eigen::Matrix<float,3,4> foot_pos_rel_last_time;
        Eigen::Matrix<float,3,4> foot_pos_target_last_time;
        Eigen::Matrix<float,3,4> foot_pos_error;
        Eigen::Matrix<float,3,4> foot_vel_error;
        Eigen::Matrix<float,3,4> lastLegTorque;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H
