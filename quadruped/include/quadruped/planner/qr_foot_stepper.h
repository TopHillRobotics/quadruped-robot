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

#ifndef QR_FOOT_STEPPER_H_
#define QR_FOOT_STEPPER_H_

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <queue>

#include "quadprogpp/QuadProg++.hh"
#include "quadprogpp/Array.hh"
#include "robots/qr_robot.h"
#include "common/qr_se3.h"
#include "state_estimator/qr_ground_estimator.h"


class qrFootStepper  {
public:
    qrFootStepper (qrTerrain& terrain, float defaultFootholdOffset, std::string level);

    void Reset(float timeSinceReset) {}

    inline Eigen::Matrix<float, 3, 1> GetDefaultFootholdOffset(int legId)
    {
        return {defaultFootholdDelta, 0.f, 0.f};
    }

    /**
     * @brief Find a optimal foot placement for swing legs, usually larger then zero.
     * @param Matrix<float, 3, 4> feet positions in world frame when all stance at ground.
     * @note Assuming that foot offset L = L0 + x, gap width is W,
     * the cost objective is F = x^T * G * x + a^T * x = x^2,
     * this means we want the increment for default offset to be small.
     * the constrain inequalities is denoted as :C^T * x >= b
     * 
     * Case 1: if front leg is possible to meet the gap with default offset,
     * then x should statifies condition: L0 - d(foot, center of gap) + x >= W/2 or <=-W/2;
     * This means the front leg either (1.a)walk through the gap or (1.b)not, respectively.
     * At the mean time, the back legs do not walk over the gap.
     */
    Eigen::Matrix<float, 3, 4> GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds);

    
    /**
     * @brief compute desired foot-end position in walk mode
     * @param currentFootholds current foot-end position of all the leg
     * @param currentComPose current com postion and pose
     * @param desiredComPose desired com postion and pose
     * @param legIds the order of legs
     */
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
        /**
     * @brief the size of gaps
     */
    std::vector<qrGap> gaps;

    /**
     * @brief describe stair information of the map
     */
    qrStair stairUp, stairDown;

    /**
     * @brief contains the step offset alone x-axis for legs in future to pass the plum piles. 
     */
    std::queue<Eigen::Matrix<float, 1, 4>> steps; 

    /**
     * @brief if false means has not generate the step
     */
    bool generatorFlag = false;

    /**
     * @brief if true means has changed the gait
     */
    bool gaitFlag = false;

    /**
     * @brief are any feet meet gap
     */
    bool meetGap;

    /**
     * @brief default foot-end position delta
     */
    float defaultFootholdDelta;

    /**
     * @brief the foot-end position delta for the next step
     */
    Eigen::Matrix<float, 3, 4> nextFootholdsOffset;


    Eigen::Matrix<float, 3, 4> lastFootholdsOffset;

    /**
     * @brief offset along Z-axis
     */
    Vec4<float> dZ;

    /**
     * @brief params for Qp solve 
     */
    quadprogpp::Matrix<double> G;
    quadprogpp::Vector<double> a;
    quadprogpp::Matrix<double> CE;
    quadprogpp::Vector<double> e;
    quadprogpp::Matrix<double> CI;
    quadprogpp::Vector<double> b;
    quadprogpp::Vector<double> x;
};

#endif //QR_FOOT_STEPPER_H_
