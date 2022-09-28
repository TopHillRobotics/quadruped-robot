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

#include "planner/qr_com_planner.h"

qrComPlanner ::qrComPlanner (qrRobot *robotIn,
                            qrGaitGenerator *gaitGeneratorIn,
                            qrRobotEstimator *robotEstimatorIn)
    : robot(robotIn), gaitGenerator(gaitGeneratorIn), robotEstimator(robotEstimatorIn), delta(1.0f)
{
    Reset(0.f);
}

void qrComPlanner ::Reset(float current_time)
{
    // update the pose in world frame by estimator
    basePosition = robot->GetBasePosition();
    baseOrientation = robot->GetBaseOrientation();
    
    legState = gaitGenerator->legState;
    normalizedPhase = gaitGenerator->normalizedPhase;
    footPosition = robot->state.GetFootPositionsInBaseFrame();
    for (int i = 0; i < 4; ++i) {
        contactK[i] = 0.f;
        swingK[i] = 0.f;
        weightFactor[i] = 0.f;
    }
    supportPolygonVertices = Eigen::Matrix<float, 3, 4>::Zero();
    comPosInBaseFrame = Eigen::Matrix<float, 3, 1>::Zero();
    comPosInWorldFrame << basePosition[0], basePosition[1], 0.f;
}

// See the MIT paper for details:https://ieeexplore.ieee.org/document/8593885
Eigen::Matrix<float, 3, 1> qrComPlanner::Update(float current_time)
{

    legState = gaitGenerator->legState;
    normalizedPhase = gaitGenerator->normalizedPhase;
    footPosition = robot->state.GetFootPositionsInBaseFrame();

    // Compute prior contact probility and stance probility
    for (int legId = 0; legId < legState.rows(); ++legId) {
        if (legState[legId] == LegState::STANCE || legState[legId] == LegState::LOSE_CONTACT) {
            contactK[legId] = 0.5 * (std::erf(normalizedPhase[legId] / (delta * sqrt(2)))
                + std::erf((1.0 - normalizedPhase[legId]) / (delta * sqrt(2)))
            );
            swingK[legId] = 0.f;
        } else {
            swingK[legId] = 0.5 * (2.0 + std::erf(-normalizedPhase[legId] / (delta * sqrt(2)))
                + std::erf((normalizedPhase[legId] - 1.0) / (delta * sqrt(2)))
            );
            contactK[legId] = 0.f;
        }
        weightFactor[legId] = contactK[legId] + swingK[legId]; // probability of leg contact
    }

    for (int legId = 0; legId < legState.rows(); ++legId) {
        auto p = footPosition.col(legId); // position of current leg
        auto footMap = ADJEST_LEG[legId]; // id of adjacent legs
        auto pCw = footPosition.col(footMap["cw"]); // position of CW foot
        auto pCcw = footPosition.col(footMap["ccw"]); // position of CCW foot

        auto phi = weightFactor[legId];
        auto phiCw = weightFactor[footMap["cw"]];
        auto phiCcw = weightFactor[footMap["ccw"]];

        Eigen::Matrix<float, 6, 2> pMat = Eigen::Matrix<float, 6, 2>::Zero();
        pMat.block<3, 1>(0, 0) = p; 
        pMat.block<3, 1>(3, 0) = p;
        pMat.block<3, 1>(0, 1) = pCw;
        pMat.block<3, 1>(3, 1) = pCcw;
        Eigen::Matrix<float, 2, 1> phiMat = {phi, 1 - phi};
        auto virtualPointAdj = pMat * phiMat; // shape = 6 * 1
        Eigen::Matrix<float, 3, 1> virtualPointCw = virtualPointAdj.head(3);
        Eigen::Matrix<float, 3, 1> virtualPointCcw = virtualPointAdj.tail(3);
        supportPolygonVertices.col(legId) = (phi * p + phiCcw * virtualPointCcw + phiCw * virtualPointCw)
            / (phi + phiCcw + phiCw);
    }

    comPosInBaseFrame = supportPolygonVertices.rowwise().mean(); // shape = (3,1)

    return comPosInBaseFrame;
}
