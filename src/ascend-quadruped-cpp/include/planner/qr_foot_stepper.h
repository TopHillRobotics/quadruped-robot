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

#ifndef QR_FOOT_STEPPER_H_
#define QR_FOOT_STEPPER_H_

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <queue>

#include "QuadProg++.hh"
#include "Array.hh"
#include "robots/qr_robot.h"
#include "utils/se3.h"
#include "state_estimator/qr_ground_estimator.h"

namespace Quadruped {
    
    // todo :different terrains may apply to different qrFootStepper , using factory method.
    class qrFootStepper  {
    public:
        qrFootStepper (qrTerrain& terrain, float defaultFootholdOffset, std::string level);

        void Reset(float timeSinceReset) {}

        inline Eigen::Matrix<float, 3, 1> GetDefaultFootholdOffset(int legId)
        {
            return {defaultFootholdDelta, 0.f, 0.f}; // todo : 1 DIM
        }

        /**
         * @brief Find a optimal foot placement for swing legs, usually larger then zero.
         * @param Eigen::Matrix<float, 3, 4> feet positions in world frame when all stance at ground.
         * @note Assuming that foot offset L = L0 + x, gap width is W,
         *         the cost objective is F = x^T * G * x + a^T * x = x^2,
         *          this means we want the increment for default offset to be small.
         *        the constrain inequalities is denoted as :
         *          C^T * x >= b
         *      Case 1: if front leg is possible to meet the gap with default offset,
         *              then x should statifies condition: L0 - d(foot, center of gap) + x >= W/2 or <=-W/2;
         *              This means the front leg either (1.a)walk through the gap or (1.b)not, respectively.
         *              At the mean time, the back legs DO NOT walk over the gap.
         *              To express (1.b) in matrix form,
         *                  [1, -1, -1, -1, -1]^T * x = [x   >= b = [ -L0
         *                                               -x
         *                                               -x           L0-p(gap)+p(foot)
         *                                               -x
         *                                               -x]         ]
         */
        Eigen::Matrix<float, 3, 4> GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds);

        std::tuple<Eigen::Matrix<float,3,4>, Eigen::Matrix<float,3,4>> GetFootholdsInWorldFrame(
                                                        Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                        Eigen::Matrix<float, 6, 1>& currentComPose,
                                                        Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                        std::vector<int>& legIds);
        /**
         * @brief Find a optimal placement for swing legs, and check it.
         * @param currentFootholdsX
         * @param front: whether the front legs cross the gap, 1.0： true, -1.0: false
         * @param back: whether the back legs cross the gap
         * @param frontGap
         * @param backGap
         * @return if solution valid, return x; otherwise, return -1;
         */        
        double CheckSolution(Eigen::Matrix<float, 1, 4> currentFootholdsX, double front, double back, qrGap frontGap, qrGap backGap);        

        /**
         * @brief genetator the next step offset alone x-axis for legs  to pass the plum piles.
         * @return int,0: success, -1: adjust previous step, -2: no invalid solution.
         */
        int StepGenerator(Eigen::Matrix<float, 1, 4>& currentFootholds, Eigen::Matrix<float, 1, 4>& desiredFootholdsOffset);

    protected:
        std::vector<qrGap> gaps;
        qrStair stairUp, stairDown;
        std::queue<Eigen::Matrix<float, 1, 4>> steps; // contains the step offset alone x-axis for legs in future to pass the plum piles.
        bool generatorFlag = false; // if false means has not generate the step
        bool gaitFlag = false; // if true means has changed the gait
        bool meetGpa; // are any feet meet gap?
        float defaultFootholdDelta;
        Eigen::Matrix<float, 3, 4> nextFootholdsOffset;
        Eigen::Matrix<float, 3, 4> lastFootholdsOffset;
        Vec4<float> dZ; // offset along Z-axis
        Vec4<int> count = Vec4<int>::Zero();
        // for QP param
        quadprogpp::Matrix<double> G;
        quadprogpp::Vector<double> a;
        quadprogpp::Matrix<double> CE;
        quadprogpp::Vector<double> e;
        quadprogpp::Matrix<double> CI;
        quadprogpp::Vector<double> b;
        quadprogpp::Vector<double> x;
    };
} // namespace Quadruped
#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_FOOT_STEPPER_H_
