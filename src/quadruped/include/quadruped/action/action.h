// The MIT License

// Copyright (c) 2022
// qrRobot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

#ifndef QR_ACTION_H
#define QR_ACTION_H

#include <math.h>
#include <algorithm>
#include "robots/qr_robot.h"
#include "mpc_controller/qr_locomotion_controller.h"

namespace Quadruped {
    namespace Action {
        
        void StandUp(qrRobot *robot, float standUpTime, float totalTime, float timeStep);

        void SitDown(qrRobot *robot, float sitDownTime, float timeStep);

        void KeepStand(qrRobot *robot, float KeepStandTime = 1.0, float timeStep = 0.001);
    
        void ControlFoot(qrRobot *robot, qrLocomotionController *locomotionController, float walkTime, float timeStep);
        
    } // Action
} // Quadruped
#endif //QR_ACTION_H
