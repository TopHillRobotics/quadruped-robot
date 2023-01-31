/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*           
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_FOOT_STEPPER_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_FOOT_STEPPER_H_

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <tuple>
#include <queue>

#include "QuadProg++.hh"
#include "Array.hh"
#include "robots/robot.h"
#include "utils/se3.h"
#include "estimators/state_estimator.hpp"

namespace Quadruped {
    
    // todo :different terrains may apply to different FootStepper, using factory method.
    class FootStepper {
    public:
        FootStepper(Terrain& terrain, float defaultFootholdOffset, std::string level);

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
        double CheckSolution(Eigen::Matrix<float, 1, 4> currentFootholdsX, double front, double back, Gap frontGap, Gap backGap);        

        /**
         * @brief genetator the next step offset alone x-axis for legs  to pass the plum piles.
         * @return int,0: success, -1: adjust previous step, -2: no invalid solution.
         */
        int StepGenerator(Eigen::Matrix<float, 1, 4>& currentFootholds, Eigen::Matrix<float, 1, 4>& desiredFootholdsOffset);

    protected:
        std::vector<Gap> gaps;
        Stair stairUp, stairDown;
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
