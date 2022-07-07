// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
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

#ifndef QR_COM_PLANNER_H
#define QR_COM_PLANNER_H

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cmath>

#include "qr_gait_generator.h"
#include "common/qr_se3.h"
#include "robot/qr_robot.h"

    class qrComPlanner {
    public:
        qrComPlanner(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn);
        virtual ~qrComPlanner() = default;

        /**
         * @brief Called during the start of a controller.
         * @param current_time: The wall time in seconds.
         */
        void Reset(float currentTime);

        Vec3<float> Update(float currentTime);

        /**
         * @brief
         * @return Eigen::Matrix<float,3,1>: comPosInBaseFrame
         */
        Vec3<float> GetComPosInBaseFrame();

        /**
         * @brief the vector index of ADJEST_LEG means the order of the legs,
         *        the value of ADJEST_LEG means adjacent two legs of the indexed leg,
         *        where cw means clockwise leg, the ccw means counter-clockwise leg.
         */
        const std::vector<std::map<std::string, int>> ADJEST_LEG{std::map<std::string, int>{{"cw", 2}, {"ccw", 1}},
                                                                 std::map<std::string, int>{{"cw", 0}, {"ccw", 3}},
                                                                 std::map<std::string, int>{{"cw", 3}, {"ccw", 0}},
                                                                 std::map<std::string, int>{{"cw", 1}, {"ccw", 2}}};


        qrRobotState *robotState;

        qrRobotConfig *robotConfig;

        qrGaitGenerator *gaitGenerator;

        /**
         * @brief com position from robot state
         */
        Vec3<float> basePosition;

        /**
         * @brief com pose from robot state
         */
        Vec4<float> baseOrientation;

        /**
         * @brief com positon after ajusted in base frame
         */
        Vec3<float> comPosInBaseFrame;

        /**
         * @brief com position after ajusted in world frame
         *
         */
        Vec3<float> comPosInWorldFrame;

        /**
         * @brief the state of each leg from qrGaitGenerator class
         * e.g. SWING/STAND
         */
        Vec4<int> legState;

        /**
         * @brief the relative phase for the desired state.
         */
        Vec4<float> normalizedLegPhase;

        /**
         * @brief the foot-end position from robot state in base frame
         */
        Mat3x4<float> footPosition;

        /**
         * @brief the probability of foot end touching the ground(range:0.~1.)
         */
        float contactK[4];

        /**
         * @brief the probability of each leg being the swing leg
         */
        float swingK[4];

        /**
         * @brief the probability of leg contact
         * i.e weightFactor[legId] = contactK[legId] + swingK[legId]
         */
        float weightFactor[4];

        /**
         * @brief coordinates of each point in the support polygon
         * 
         */
        Mat3x4<float> supportPolygonVertices;

        /**
         * @brief used to caculate contactK and swingK
         */
        float delta;
    };

#endif // QR_COM_PLANNER_H
