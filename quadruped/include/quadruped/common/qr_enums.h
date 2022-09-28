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

#ifndef QR_TYPES_H
#define QR_TYPES_H
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
    WALK_LOCOMOTION,
    ADVANCED_TROT_LOCOMOTION
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

enum GaitType {
    STAND,
    STATIC_WALK,
    AMBLE,
    TROT
};
#endif //QR_TYPES_H
