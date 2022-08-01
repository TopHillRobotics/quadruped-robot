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
namespace Quadruped {
    enum MotorMode {
        POSITION_MODE,
        TORQUE_MODE,
        HYBRID_MODE
    };

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
    enum SubLegState { // for walk
        LOAD_FORCE=5,
        UNLOAD_FORCE,
        FULL_STANCE,
        TRUE_SWING,
    };

/** @brief high level control mode */
    enum LocomotionMode {
        VELOCITY_LOCOMOTION,
        POSITION_LOCOMOTION,
        WALK_LOCOMOTION
    };

    enum TerrainType {
        PLANE=0,
        PLUM_PILES,
        STAIRS
    };

/** @brief main function gets vel commands by which mode */
    enum TwistMode {
        CONST,
        ROS,
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TYPES_H
