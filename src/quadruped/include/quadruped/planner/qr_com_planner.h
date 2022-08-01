
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

#ifndef QR_COM_PLANNER_H
#define QR_COM_PLANNER_H

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cmath>

#include "robots/qr_robot.h"
#include "state_estimator/qr_robot_estimator.h"
#include "mpc_controller/qr_gait_generator.h"
#include "utils/se3.h"

namespace Quadruped {
    class qrComPlanner  {
    private:
        /**
         * @brief the vector index of ADJEST_LEG means the order of the legs,
         *        the value of ADJEST_LEG means adjacent two legs of the indexed leg, 
         *        where cw means clockwise leg, the ccw means counter-clockwise leg.
         */         
        const std::vector<std::map<std::string, int>> ADJEST_LEG{std::map<std::string, int>{{"cw", 2}, {"ccw", 1}},
                                                                 std::map<std::string, int>{{"cw", 0}, {"ccw", 3}},
                                                                 std::map<std::string, int>{{"cw", 3}, {"ccw", 0}},
                                                                 std::map<std::string, int>{{"cw", 1}, {"ccw", 2}}
        };

        qrRobot *robot;
        qrGaitGenerator *gaitGenerator;
        qrRobotEstimator *robotEstimator;

        Eigen::Vector3f basePosition;
        Eigen::Vector4f baseOrientation;
        Eigen::Vector3f inverseTranslation;
        Eigen::Matrix3f inverseRotation;
        Eigen::Matrix<float, 3, 1> comPosInBaseFrame;
        Eigen::Matrix<float, 3, 1> comPosInWorldFrame;
        Eigen::Matrix<int, 4, 1> legState;
        Eigen::Matrix<float, 4, 1> normalizedPhase;
        Eigen::Matrix<float, 3, 4> footPosition; // in base frame
        float contactK[4]; // is the foot contact with ground.
        float swingK[4]; // is it a swing foot ?
        float weightFactor[4]; // weight factors of vertices.
        Eigen::Matrix<float, 3, 4> supportPolygonVertices;
        float delta;

    public:
        qrComPlanner (qrRobot *robotIn,
                    qrGaitGenerator *gaitGeneratorIn,
                    qrRobotEstimator *robotEstimatorIn);

        ~qrComPlanner () = default;

        /**
         * @brief Called during the start of a controller.
         * @param current_time: The wall time in seconds.
         */
        void Reset(float currentTime);

        Eigen::Matrix<float, 3, 1> Update(float currentTime);

        Eigen::Matrix<float, 3, 1> TestUpdate(float currentTime);

        inline Eigen::Matrix<float, 3, 1> &GetComPosInBaseFrame()
        {
            return comPosInBaseFrame;
        }
    };
} //namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_COM_ADJUSTER_H_
