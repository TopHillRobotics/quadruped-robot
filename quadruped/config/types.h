/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot runtime used to lanuch the robot
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       add low/high level mode @ Zhu Yijie 2021-11-24;
*/
#ifndef ASCEND_QUADRUPED_CPP_TYPES_H
#define ASCEND_QUADRUPED_CPP_TYPES_H
#include <stdio.h>

namespace Quadruped {
    /*!
    * Link indices for cheetah-shaped robots
    */
    namespace linkID {
        constexpr size_t FR = 9;   // Front Right Foot
        constexpr size_t FL = 11;  // Front Left Foot
        constexpr size_t HR = 13;  // Hind Right Foot
        constexpr size_t HL = 15;  // Hind Left Foot

        constexpr size_t FR_abd = 2;  // Front Right Abduction
        constexpr size_t FL_abd = 0;  // Front Left Abduction
        constexpr size_t HR_abd = 3;  // Hind Right Abduction
        constexpr size_t HL_abd = 1;  // Hind Left Abduction
    }  // namespace linkID

    enum MotorMode {
        POSITION_MODE,
        TORQUE_MODE,
        HYBRID_MODE
    };

    /** @brief motor command composite*/
    enum HybridCmd {
        POSITION,
        KP,
        VELOCITY,
        KD,
        TORQUE
    };

    enum LegState {
        SWING=0,
        STANCE,
        EARLY_CONTACT,
        LOSE_CONTACT,
        USERDEFINED_SWING,
    };
    
    /** @brief for walk mode */
    enum SubLegState {
        LOAD_FORCE=5,
        UNLOAD_FORCE,
        FULL_STANCE,
        TRUE_SWING,
    };

    /** @brief high level control mode */
    enum LocomotionMode {
        VELOCITY_LOCOMOTION,
        POSITION_LOCOMOTION,
        WALK_LOCOMOTION,
        ADVANCED_TROT
    };

    enum TerrainType {
        PLANE=0,
        PLUM_PILES,
        STAIRS,
        SLOPE,
        ROUGH
    };

    /** @brief main function gets vel commands by which mode */
    enum TwistMode {
        CONST,
        ROS,
    };

    /** @brief  used for remote controll command */
    enum RC_MODE {
        HARD_CODE = 0,
        JOY_TROT,
        JOY_ADVANCED_TROT,
        JOY_WALK,
        JOY_STAND,
        // ROS_WALK
        // VISUAL,
        BODY_UP,
        BODY_DOWN,
        EXIT,
        RC_MODE_ITEMS = EXIT
    };

} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TYPES_H
