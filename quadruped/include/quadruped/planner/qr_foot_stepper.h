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

#ifndef QR__FOOT_STEPPER_H
#define QR__FOOT_STEPPER_H

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <queue>

#include "QuadProg++.hh"
#include "Array.hh"
#include "robots/qr_robot.h"
#include "utils/qr_se3.h"
#include "estimators/qr_state_estimator_container.h"


namespace Quadruped {

/**
 * @brief The FootStepper class
 * @todo different terrains may apply to different FootStepper, using factory method.
 */
class qrFootStepper {

public:

    /**
     * @brief Constructor of FootStepper class.
     * @param terrain: the 3D plane that robot contact.
     * @param defaultFootholdOffset: default contact point offset to centrol of foot.
     * @param level: terrian stairs level.
     */
    qrFootStepper(qrTerrain& terrain, float defaultFootholdOffset, std::string level);

    void Reset(float timeSinceReset) {
    };

    /**
     * @brief Get default foot-end position delta.
     * @param legId: which leg.
     * @return default foot-end position delta.
     */
    inline Eigen::Matrix<float, 3, 1> GetDefaultFootholdOffset(int legId) {
        return {defaultFootholdDelta, 0.f, 0.f}; // todo : 1 DIM
    };

    /**
     * @brief Find a optimal foot placement for swing legs, usually larger then zero.
     * @param Eigen::Matrix<float, 3, 4> feet positions in world frame when all stance at ground.
     * @note Assuming that foot offset L = L0 + x, gap width is W,
     *       the cost objective is F = x^T * G * x + a^T * x = x^2,
     *       this means we want the increment for default offset to be small.
     *       the constrain inequalities is denoted as :
     *           C^T * x >= b
     *       Case 1: if front leg is possible to meet the gap with default offset,
     *       then x should statifies condition: L0 - d(foot, center of gap) + x >= W/2 or <=-W/2;
     *       This means the front leg either (1.a)walk through the gap or (1.b)not, respectively.
     *       At the mean time, the back legs DO NOT walk over the gap.
     *       To express (1.b) in matrix form,
     *           [1, -1, -1, -1, -1]^T * x = [x >= b = [ -L0
     *                                       -x
     *                                       -x           L0-p(gap)+p(foot)
     *                                       -x
     *                                       -x]         ]
     */
    Eigen::Matrix<float, 3, 4> GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds);

    /**
     * @brief Compute next footholds in world frame.
     * @param currentFootholds: current footholds in world frame.
     * @param currentComPose: current CoM pose.
     * @param desiredComPose: desired CoM pose.
     * @param legIds: the leg id.
     * @return next footholds in world frame.
     */
    std::tuple<Eigen::Matrix<float,3,4>, Eigen::Matrix<float,3,4>> GetFootholdsInWorldFrame(
                                                    Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                    Eigen::Matrix<float, 6, 1>& currentComPose,
                                                    Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                    std::vector<int>& legIds);

    /**
     * @brief Find a optimal placement for swing legs, and check it.
     * @param currentFootholdsX: current footholds x position.
     * @param front: whether the front legs cross the gap, 1.0： true, -1.0: false.
     * @param back: whether the back legs cross the gap.
     * @param frontGap: front gap
     * @param backGap: back gap
     * @return if solution valid, return x; otherwise, return -1;
     */        
    double CheckSolution(Eigen::Matrix<float, 1, 4> currentFootholdsX, double front, double back, qrGap frontGap, qrGap backGap);        

    /**
     * @brief Genetator the next step offset alone x-axis for legs  to pass the plum piles.
     * @return int,0: success, -1: adjust previous step, -2: no invalid solution.
     */
    int StepGenerator(Eigen::Matrix<float, 1, 4>& currentFootholds, Eigen::Matrix<float, 1, 4>& desiredFootholdsOffset);

protected:

    /**
     * @brief The size of gaps.
     */
    std::vector<qrGap> gaps;

    /**
     * @brief Describe stair information of the map.
     */
    qrStair stairUp, stairDown;

    /**
     * @brief Contains the step offset alone x-axis for legs in future to pass the plum piles.
     */
    std::queue<Eigen::Matrix<float, 1, 4>> steps; // contains the step offset alone x-axis for legs in future to pass the plum piles.

    /**
     * @brief If false means has not generate the step.
     */
    bool generatorFlag = false; // if false means has not generate the step

    /**
     * @brief If true means has changed the gait.
     */
    bool gaitFlag = false; // if true means has changed the gait

    /**
     * @brief Are any feet meet gap.
     */
    bool meetGpa; // are any feet meet gap?

    /**
     * @brief Default foot-end position delta.
     */
    float defaultFootholdDelta;

    /**
     * @brief The foot-end position delta for the next step.
     */
    Eigen::Matrix<float, 3, 4> nextFootholdsOffset;

    /**
     * @brief The foot-end position delta for the last step.
     */
    Eigen::Matrix<float, 3, 4> lastFootholdsOffset;

    /**
     * @brief Offset along Z-axis.
     */
    Vec4<float> dZ;

    /**
     * @brief params for Qp solve:
     * min 0.5* x G x + g0 x
     * s.t.
     * CE^T x + ce0 = 0
     * CI^T x + ci0 >= 0
     */
    quadprogpp::Matrix<double> G;

    quadprogpp::Vector<double> a;

    quadprogpp::Matrix<double> CE;

    quadprogpp::Vector<double> e;

    quadprogpp::Matrix<double> CI;

    quadprogpp::Vector<double> b;

    /**
     * @brief result of qp problem
     */
    quadprogpp::Vector<double> x;

};

} // Namespace Quadruped

#endif // QR_FOOT_STEPPER_H
