// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_ENUM_TYPES_H
#define QR_ENUM_TYPES_H

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
}  // Namespace linkID

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

} // Namespace Quadruped

#endif //QR_ENUM_TYPES_H
